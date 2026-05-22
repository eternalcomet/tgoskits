//! SDHCI 中断处理模块  
//!  
//! 设计模式：
//!   - ISR: 裸函数，操作 Atomic + MMIO，不持锁、不分配、不调度、不 log  
//!   - 正常上下文: CviSdhci 的 wait_*/try_* 方法消费 ISR 设置的标志  
//!   - 上层: 通过 PollSet + poll_fn 组装异步等待  
//!  
//! ISR 处理的事件:  
//!   - CMD_COMPLETE (bit 0): W1C + 设标志  
//!   - XFER_COMPLETE (bit 1): W1C + 设标志  
//!   - BUF_WR_READY (bit 4): W1C + 设标志  
//!   - BUF_RD_READY (bit 5): W1C + 设标志  
//!   - CARD_INT (bit 8): mask 信号 + 调用回调  
//!   - ERROR (bit 15): 读 ERR_STS → 存 error_status + 设标志  

use core::{
    sync::atomic::{AtomicBool, AtomicU16, AtomicU64, AtomicUsize, Ordering},
    task::Waker,
};

use axpoll::PollSet;

use crate::{mmio_read, mmio_write, regs::*};

macro_rules! handle_event {
    ($norm:expr, $w1c:expr, $bit:expr, $flag:expr) => {
        if $norm & $bit != 0 {
            $flag.store(true, Ordering::Release);
            $w1c |= $bit;
        }
    };
}

/// SDHCI 中断全局状态
///
/// ISR 通过此结构体与 CviSdhci 的 wait_* 方法通信。
/// 所有字段均为 Atomic，ISR 安全。
struct SdhciIrqState {
    /// SDHCI MMIO 基地址（ISR 裸写用）
    base: AtomicUsize,
    /// CARD_INT 回调（通知上层驱动有数据可读）
    card_irq_callback: AtomicUsize, // fn() 的裸指针
    /// CMD Complete 标志
    cmd_complete: AtomicBool,
    /// Transfer Complete 标志
    xfer_complete: AtomicBool,
    /// Buffer Read Ready 标志
    buf_rd_ready: AtomicBool,
    /// Buffer Write Ready 标志
    buf_wr_ready: AtomicBool,
    /// Error 标志
    error: AtomicBool,
    /// Error Interrupt Status 值（ISR 读取后存入）
    error_status: AtomicU16,
}

impl SdhciIrqState {
    const fn new() -> Self {
        Self {
            base: AtomicUsize::new(0),
            card_irq_callback: AtomicUsize::new(0),
            cmd_complete: AtomicBool::new(false),
            xfer_complete: AtomicBool::new(false),
            buf_rd_ready: AtomicBool::new(false),
            buf_wr_ready: AtomicBool::new(false),
            error: AtomicBool::new(false),
            error_status: AtomicU16::new(0),
        }
    }
}

/// 全局 ISR 状态
static SDHCI_IRQ_STATE: SdhciIrqState = SdhciIrqState::new();
static SDHCI_POLLSET: PollSet = PollSet::new();

pub static SDHCI_IRQ_COUNT: AtomicU64 = AtomicU64::new(0);
pub static SDHCI_LAST_NORM: AtomicU16 = AtomicU16::new(0);
pub static SDHCI_CARD_INT_COUNT: AtomicU64 = AtomicU64::new(0);

/// 初始化 ISR 全局状态（设置 MMIO 基地址）
pub fn irq_state_init(base: usize) {
    SDHCI_IRQ_STATE.base.store(base, Ordering::Release);
}

/// 注册 CARD_INT 回调函数
///
/// WiFi 驱动初始化时调用，注册一个函数用于在 ISR 中通知"卡有数据可读"。
/// 回调在硬中断上下文执行，禁止：持锁、分配堆、调度、调用 log。
pub fn register_card_irq_callback(cb: fn()) {
    SDHCI_IRQ_STATE
        .card_irq_callback
        .store(cb as usize, Ordering::Release);
}

/// 使能所有 SDHCI 中断信号（切换到中断驱动模式）
///
/// 使能后，CMD_COMPLETE / XFER_COMPLETE / BUF_RD_READY / BUF_WR_READY / CARD_INT
/// 会触发 PLIC 中断，由 `sdhci_irq_handler` 处理。
/// `CviSdhci` 的 `try_*` 方法通过检查 AtomicBool 获取结果。
pub fn enable_irq_signals() {
    let base = SDHCI_IRQ_STATE.base.load(Ordering::Acquire);
    mmio_write::<u16>(base + SDHCI_NORM_INT_SIG_EN as usize, NORM_INT_SIG_MASK);
    mmio_write::<u16>(base + SDHCI_ERR_INT_SIG_EN as usize, ERR_INT_SIG_MASK);
}

/// 禁用所有 SDHCI 中断信号（回到轮询模式）
pub fn disable_irq_signals() {
    let base = SDHCI_IRQ_STATE.base.load(Ordering::Acquire);
    mmio_write::<u16>(base + SDHCI_NORM_INT_SIG_EN as usize, 0);
    mmio_write::<u16>(base + SDHCI_ERR_INT_SIG_EN as usize, 0);
}

/// 屏蔽/恢复 CARD_INT 信号（裸地址操作，ISR 安全）
pub(crate) fn mask_card_irq_raw(base: usize, mask: bool) {
    let addr = base + SDHCI_NORM_INT_SIG_EN as usize;
    let cur = mmio_read::<u16>(addr);
    mmio_write::<u16>(
        addr,
        (cur & !NORM_INT_CARD_INT) | (!mask as u16 * NORM_INT_CARD_INT),
    );
}

/// 检查并消费 CMD_COMPLETE 标志
pub(crate) fn take_cmd_complete() -> bool {
    SDHCI_IRQ_STATE.cmd_complete.swap(false, Ordering::AcqRel)
}

/// 检查并消费 XFER_COMPLETE 标志
pub(crate) fn take_xfer_complete() -> bool {
    SDHCI_IRQ_STATE.xfer_complete.swap(false, Ordering::AcqRel)
}

/// 检查并消费 BUF_RD_READY 标志
pub(crate) fn take_buf_rd_ready() -> bool {
    SDHCI_IRQ_STATE.buf_rd_ready.swap(false, Ordering::AcqRel)
}

/// 检查并消费 BUF_WR_READY 标志
pub(crate) fn take_buf_wr_ready() -> bool {
    SDHCI_IRQ_STATE.buf_wr_ready.swap(false, Ordering::AcqRel)
}

/// 检查并消费 ERROR 标志，返回错误状态码
pub(crate) fn take_error() -> Option<u16> {
    if SDHCI_IRQ_STATE.error.swap(false, Ordering::AcqRel) {
        Some(SDHCI_IRQ_STATE.error_status.swap(0, Ordering::AcqRel))
    } else {
        None
    }
}

pub(crate) fn drain_flags() {
    take_cmd_complete();
    take_xfer_complete();
    take_buf_rd_ready();
    take_buf_wr_ready();
    take_error();
}

/// 注册 waker，等待 ISR 唤醒
#[allow(dead_code)]
pub(crate) fn register_sdhci_waker(waker: &Waker) {
    SDHCI_POLLSET.register(waker);
}

/// SDHCI 中断处理函数（注册到 PLIC）
///
/// 约束：不持锁、不分配堆、不调度、不调用 log。
pub fn sdhci_irq_handler(_irq: usize) {
    SDHCI_IRQ_COUNT.fetch_add(1, Ordering::Relaxed);

    let base = SDHCI_IRQ_STATE.base.load(Ordering::Acquire);
    if base == 0 {
        return;
    }

    let status = mmio_read::<u32>(base + SDHCI_INT_STATUS_NORM as usize);
    if status == 0 {
        return;
    }

    let (norm, err) = (status as u16, (status >> 16) as u16);
    SDHCI_LAST_NORM.store(norm, Ordering::Relaxed);
    let has_error = norm & NORM_INT_ERROR != 0;

    // ---- ERROR ----
    if has_error {
        // 只清 ERR_STATUS；NORM bit15 是只读汇总位，硬件自动反映
        mmio_write::<u16>(base + SDHCI_INT_STATUS_ERR as usize, err);
        SDHCI_IRQ_STATE.error_status.store(err, Ordering::Release);
        SDHCI_IRQ_STATE.error.store(true, Ordering::Release);
    }

    let mut w1c_bits: u16 = 0;
    // ---- 完成事件（表驱动）----
    if !has_error {
        handle_event!(
            norm,
            w1c_bits,
            NORM_INT_CMD_COMPLETE,
            SDHCI_IRQ_STATE.cmd_complete
        );
        handle_event!(
            norm,
            w1c_bits,
            NORM_INT_XFER_COMPLETE,
            SDHCI_IRQ_STATE.xfer_complete
        );
        handle_event!(
            norm,
            w1c_bits,
            NORM_INT_BUF_WR_READY,
            SDHCI_IRQ_STATE.buf_wr_ready
        );
        handle_event!(
            norm,
            w1c_bits,
            NORM_INT_BUF_RD_READY,
            SDHCI_IRQ_STATE.buf_rd_ready
        );
    } else {
        // ERROR 时也要 W1C 清除这些位，但不设标志
        w1c_bits = norm
            & (NORM_INT_CMD_COMPLETE
                | NORM_INT_XFER_COMPLETE
                | NORM_INT_BUF_WR_READY
                | NORM_INT_BUF_RD_READY);
    }

    // 一次性 W1C 清除所有 Normal 位（含 ERROR 汇总位）
    if w1c_bits != 0 {
        mmio_write::<u16>(base + SDHCI_INT_STATUS_NORM as usize, w1c_bits);
    }

    // 唤醒等待者
    if has_error || w1c_bits != 0 {
        SDHCI_POLLSET.wake();
    }

    // ---- CARD_INT（电平触发，独立处理）----
    if norm & NORM_INT_CARD_INT != 0 {
        SDHCI_CARD_INT_COUNT.fetch_add(1, Ordering::Relaxed);
        mask_card_irq_raw(base, true);
        let cb = SDHCI_IRQ_STATE.card_irq_callback.load(Ordering::Acquire);
        if cb != 0 {
            unsafe { core::mem::transmute::<usize, fn()>(cb)() };
        }
    }
}
