//! CVI SoC (CV1800/SG2002) SDHCI 控制器驱动  
//!  
//! 职责:  
//!   - SDHCI 标准寄存器操作 (CMD52/CMD53/PIO)  
//!   - SDIO 卡枚举 (CMD5/CMD3/CMD7)  
//!   - 中断处理 (ISR + AtomicBool 标志)  
//!   - 时钟/电源/总线宽度配置  
//!  
//! 设计:  
//!   - init 阶段: 轮询 MMIO 寄存器 (ISR 未注册)  
//!   - 运行阶段: ISR 做 W1C + 设标志, wait_* 检查 AtomicBool  
//!   - 上层可用 irq::try_take_* + poll_fn 组装异步等待  

#![no_std]

extern crate alloc;

pub mod hw_init;
pub mod irq;
pub mod regs;

use alloc::sync::Arc;
use core::ptr::{read_volatile, write_volatile};

use sdio_host::{SdioCardIrq, SdioHost, cccr::*, cmd::*, error::SdioError};

use crate::regs::*;

pub(crate) fn mmio_read<T: Copy>(addr: usize) -> T {
    unsafe { read_volatile(addr as *const T) }
}

pub(crate) fn mmio_write<T: Copy>(addr: usize, val: T) {
    unsafe { write_volatile(addr as *mut T, val) }
}

pub struct CviCardIrqCtrl {
    base: usize,
}

impl CviCardIrqCtrl {
    pub fn new(base: usize) -> Self {
        Self { base }
    }
}

impl SdioCardIrq for CviCardIrqCtrl {
    fn mask_card_irq(&self) {
        irq::mask_card_irq_raw(self.base, true);
    }

    fn unmask_card_irq(&self) {
        irq::mask_card_irq_raw(self.base, false);
    }
}

/// CVI SoC WiFi SDIO 控制器  
pub struct CviSdhci {
    base: usize, // MMIO 基地址
    rca: u16,    // 相对卡地址
    vendor_id: u16,
    device_id: u16,
}

impl CviSdhci {
    pub fn new(base_addr: usize) -> Self {
        Self {
            base: base_addr,
            rca: 0,
            vendor_id: 0,
            device_id: 0,
        }
    }

    #[inline(always)]
    fn read<T: Copy>(&self, off: u32) -> T {
        mmio_read::<T>(self.base + off as usize)
    }
    #[inline(always)]
    fn write<T: Copy>(&self, off: u32, val: T) {
        mmio_write::<T>(self.base + off as usize, val)
    }

    /// 中断驱动等待（零 spin，任务睡眠，ISR 唤醒）
    fn classify_error(err: u16) -> SdioError {
        if err & ERR_INT_CMD_CRC != 0 {
            log::error!("[SDHCI] CMD CRC error (err_sts=0x{:04x})", err);
        }
        if err & ERR_INT_DAT_CRC != 0 {
            log::error!("[SDHCI] DAT CRC error (err_sts=0x{:04x})", err);
        }
        if err & ERR_INT_CMD_TIMEOUT != 0 {
            log::error!("[SDHCI] CMD timeout (err_sts=0x{:04x})", err);
        }
        if err & ERR_INT_DAT_TIMEOUT != 0 {
            log::error!("[SDHCI] DAT timeout (err_sts=0x{:04x})", err);
        }
        match err {
            e if e & (ERR_INT_CMD_CRC | ERR_INT_DAT_CRC) != 0 => SdioError::CrcError,
            e if e & (ERR_INT_CMD_TIMEOUT | ERR_INT_DAT_TIMEOUT) != 0 => SdioError::Timeout,
            _ => SdioError::IoError,
        }
    }

    /// 尝试消费一个 IRQ 标志，返回 Some(result) 表示已决，None 表示未就绪  
    fn try_take(&self, take: fn() -> bool) -> Option<Result<(), SdioError>> {
        if let Some(err) = irq::take_error() {
            self.reset_dat_line();
            return Some(Err(Self::classify_error(err)));
        }
        if take() {
            return Some(Ok(()));
        }
        None
    }

    fn wait_irq_flag(&self, take: fn() -> bool) -> Result<(), SdioError> {
        for _ in 0..1000 {
            if let Some(r) = self.try_take(take) {
                return r;
            }
            core::hint::spin_loop();
        }
        for _ in 0..100_000 {
            if let Some(r) = self.try_take(take) {
                return r;
            }
            ax_task::yield_now();
        }
        log::error!("[SDHCI] wait_irq_flag timeout");
        Err(SdioError::Timeout)
    }

    fn wait_cmd_complete(&self) -> Result<u32, SdioError> {
        self.wait_irq_flag(irq::take_cmd_complete)?;
        Ok(self.read::<u32>(SDHCI_RESPONSE))
    }

    fn wait_buffer_read_ready(&self) -> Result<(), SdioError> {
        self.wait_irq_flag(irq::take_buf_rd_ready)
    }

    fn wait_buffer_write_ready(&self) -> Result<(), SdioError> {
        self.wait_irq_flag(irq::take_buf_wr_ready)
    }

    fn wait_transfer_complete(&self) -> Result<(), SdioError> {
        self.wait_irq_flag(irq::take_xfer_complete)
    }

    /// 硬件轮询（无对应中断，只能 spin）
    fn wait_cmd_idle(&self) -> Result<(), SdioError> {
        for _ in 0..CMD_RESPONSE_TIMEOUT {
            if self.read::<u32>(SDHCI_PRESENT_STATE) & SDHCI_CMD_INHIBIT == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(SdioError::Timeout)
    }

    fn wait_clock_stable(&self) -> Result<(), SdioError> {
        for _ in 0..CLOCK_STABLE_TIMEOUT {
            if self.read::<u16>(SDHCI_CLOCK_CONTROL) & CC_INT_CLK_STABLE != 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(SdioError::Timeout)
    }

    fn wait_reset_complete(&self) -> Result<(), SdioError> {
        for _ in 0..RESET_TIMEOUT {
            if self.read::<u8>(SDHCI_SOFTWARE_RESET) == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(SdioError::Timeout)
    }

    fn reset_dat_line(&self) {
        self.write::<u8>(SDHCI_SOFTWARE_RESET, SWRST_DAT_LINE);
        for _ in 0..RESET_TIMEOUT {
            if self.read::<u8>(SDHCI_SOFTWARE_RESET) & SWRST_DAT_LINE == 0 {
                return;
            }
            core::hint::spin_loop();
        }
    }

    /// SD 命令
    fn send_cmd(&self, cmd_idx: u8, arg: u32) -> Result<u32, SdioError> {
        self.wait_cmd_idle()?;
        irq::drain_flags();

        self.write::<u32>(SDHCI_ARGUMENT, arg);
        let flags = match cmd_idx {
            0 => CMD_RESP_NONE,
            3 => CMD_FLAGS_R5, // R6 与 R5 标志相同
            5 => CMD_FLAGS_R4,
            7 => CMD_FLAGS_R1B,
            52 => CMD_FLAGS_R5,
            53 => CMD_FLAGS_R5_DATA,
            _ => return Err(SdioError::Unsupported),
        };

        self.write::<u16>(
            SDHCI_COMMAND,
            (cmd_idx as u16) << CMD_INDEX_SHIFT as u16 | flags,
        );
        self.wait_cmd_complete()
    }

    fn check_r5_response(&self, resp: u32) -> Result<u8, SdioError> {
        if resp & R5_COM_CRC_ERROR != 0 {
            log::error!("[SDHCI] R5 CRC error, resp=0x{:08x}", resp);
            return Err(SdioError::CrcError);
        }
        if resp & (R5_ILLEGAL_COMMAND | R5_FUNCTION_NUMBER | R5_OUT_OF_RANGE) != 0 {
            log::error!("[SDHCI] R5 cmd/func/range error, resp=0x{:08x}", resp);
            return Err(SdioError::IoError);
        }
        if resp & R5_ERROR != 0 {
            log::error!("[SDHCI] R5 general error, resp=0x{:08x}", resp);
            return Err(SdioError::IoError);
        }
        Ok((resp & R5_DATA_MASK) as u8)
    }

    /// CMD52
    fn cmd52(&self, func: u8, addr: u32, flags: u32, val: u8) -> Result<u8, SdioError> {
        if addr > SDIO_ADDR_MASK {
            return Err(SdioError::Unsupported);
        }
        let arg =
            flags | ((func as u32 & 0x07) << 28) | ((addr & SDIO_ADDR_MASK) << 9) | val as u32;
        let resp = self.send_cmd(52, arg)?;
        self.check_r5_response(resp)
    }

    fn cmd52_read(&self, func: u8, addr: u32) -> Result<u8, SdioError> {
        self.cmd52(func, addr, 0, 0)
    }

    fn cmd52_write(&self, func: u8, addr: u32, val: u8) -> Result<(), SdioError> {
        self.cmd52(func, addr, CMD52_RW_FLAG, val)?;
        Ok(())
    }

    /// CMD53
    /// 返回 (blk_sz, nblocks) 供 PIO 使用
    #[allow(clippy::too_many_arguments)]
    fn cmd53_xfer(
        &self,
        func: u8,
        addr: u32,
        write: bool,
        inc_addr: bool,
        block_size: u16,
        use_block: bool,
        len: usize,
    ) -> Result<(u16, u16), SdioError> {
        if addr > SDIO_ADDR_MASK || len == 0 {
            return Err(SdioError::Unsupported);
        }

        let (blk_mode, count, blk_sz) = if use_block && block_size > 0 {
            let n = len / block_size as usize;
            if n == 0 || !len.is_multiple_of(block_size as usize) {
                return Err(SdioError::Unsupported);
            }
            (true, n, block_size)
        } else {
            if len > SDIO_DEFAULT_BLOCK_SIZE as usize {
                return Err(SdioError::Unsupported);
            }
            (
                false,
                if len == SDIO_DEFAULT_BLOCK_SIZE as usize {
                    0
                } else {
                    len
                },
                len as u16,
            )
        };

        let mut arg =
            ((func as u32 & 0x07) << 28) | ((addr & SDIO_ADDR_MASK) << 9) | (count as u32 & 0x1FF);
        if write {
            arg |= CMD53_RW_FLAG;
        }
        if blk_mode {
            arg |= CMD53_BLOCK_MODE;
        }
        if inc_addr {
            arg |= CMD53_OP_CODE_INC;
        }

        let xfer_blocks = if blk_mode { count as u16 } else { 1 };
        self.write::<u16>(SDHCI_BLOCK_SIZE, blk_sz);
        self.write::<u16>(SDHCI_BLOCK_COUNT, xfer_blocks);
        self.write::<u16>(
            SDHCI_TRANSFER_MODE,
            if blk_mode {
                TM_MULTI_BLOCK | TM_BLK_CNT_EN
            } else {
                0
            } | if !write { TM_DATA_DIR_READ } else { 0 },
        );

        self.send_cmd(53, arg)?;
        Ok((blk_sz, xfer_blocks))
    }

    fn cmd53_read_fixed(
        &self,
        func: u8,
        addr: u32,
        buf: &mut [u8],
        blk_sz: u16,
        use_blk: bool,
    ) -> Result<(), SdioError> {
        let (bs, nb) = self.cmd53_xfer(func, addr, false, false, blk_sz, use_blk, buf.len())?;
        self.pio_read(buf, bs, nb)?;
        self.wait_transfer_complete()
    }

    fn cmd53_write_fixed(
        &self,
        func: u8,
        addr: u32,
        buf: &[u8],
        blk_sz: u16,
        use_blk: bool,
    ) -> Result<(), SdioError> {
        let (bs, nb) = self.cmd53_xfer(func, addr, true, false, blk_sz, use_blk, buf.len())?;
        self.pio_write(buf, bs, nb)?;
        self.wait_transfer_complete()
    }

    /// PIO 读取: 逐块等待 Buffer Read Ready → 读取 Buffer Data Port
    fn pio_read(&self, buf: &mut [u8], block_size: u16, nblocks: u16) -> Result<(), SdioError> {
        let mut offset = 0;

        for _ in 0..nblocks {
            self.wait_buffer_read_ready()?; // 等待 Buffer Read Ready 中断状态位

            // 每次从 Buffer Data Port 读 4 字节
            let words = (block_size as usize).div_ceil(4); // 向上取整
            for _ in 0..words {
                let data = self.read::<u32>(SDHCI_BUFFER);
                let byte_offset = data.to_le_bytes(); // 转换为字节数组，处理未对齐的最后一个 word
                let remaining = buf.len() - offset;
                let copy_len = core::cmp::min(4, remaining);
                buf[offset..offset + copy_len].copy_from_slice(&byte_offset[..copy_len]);
                offset += copy_len;
            }
        }

        Ok(())
    }

    /// PIO 写入: 逐块等待 Buffer Write Ready → 写入 Buffer Data Port
    fn pio_write(&self, buf: &[u8], block_size: u16, nblocks: u16) -> Result<(), SdioError> {
        let mut offset = 0;

        for _ in 0..nblocks {
            self.wait_buffer_write_ready()?; // 等待 Buffer Write Ready 中断状态位

            let words = (block_size as usize).div_ceil(4); // 向上取整
            for _ in 0..words {
                let mut data: [u8; 4] = [0; 4];
                let remaining = buf.len() - offset;
                let copy_len = core::cmp::min(4, remaining);
                data[..copy_len].copy_from_slice(&buf[offset..offset + copy_len]);
                let word = u32::from_le_bytes(data);
                self.write::<u32>(SDHCI_BUFFER, word);
                offset += copy_len;
            }
        }

        Ok(())
    }

    /// 读取 CIS 指针 (3 字节, little-endian)  
    /// func=0 从 CCCR 读, func=1..7 从 FBR 读
    fn read_cis_ptr(&self, func: u8) -> Result<u32, SdioError> {
        let base = if func == 0 {
            CCCR_CIS_POINTER
        } else {
            fbr_base(func) + FBR_CIS_PTR_OFFSET
        };
        let b0 = self.cmd52_read(0, base)? as u32;
        let b1 = self.cmd52_read(0, base + 1)? as u32;
        let b2 = self.cmd52_read(0, base + 2)? as u32;
        Ok(b0 | (b1 << 8) | (b2 << 16))
    }

    /// 遍历 CIS tuple 链，查找 CISTPL_MANFID，返回 (vendor_id, device_id)
    fn read_manfid_from_cis(&self, func: u8) -> Result<(u16, u16), SdioError> {
        let mut addr = self.read_cis_ptr(func)?;
        for _ in 0..256 {
            let tuple_code = self.cmd52_read(0, addr)?;
            if tuple_code == CISTPL_END {
                break; // 遍历结束
            }
            if tuple_code == CISTPL_NULL {
                // NULL tuple 没有 link 字段，直接跳过 1 字节
                addr += 1;
                continue;
            }
            let tuple_link = self.cmd52_read(0, addr + 1)? as u32; // link 字段: 后续 tuple 的偏移量
            if tuple_code == CISTPL_MANFID && tuple_link >= 4 {
                let v0 = self.cmd52_read(0, addr + 2)? as u16;
                let v1 = self.cmd52_read(0, addr + 3)? as u16;
                let v2 = self.cmd52_read(0, addr + 4)? as u16;
                let v3 = self.cmd52_read(0, addr + 5)? as u16;
                return Ok((v0 | (v1 << 8), v2 | (v3 << 8))); // 返回 vendor_id 和 device_id
            }
            addr += 2 + tuple_link; // 移动到下一个 tuple
        }

        Err(SdioError::Unsupported) // 没有找到 Manfid
    }

    // ========== SDIO 初始化辅助函数 ==========

    /// SoC 级硬件初始化（如果需要）
    #[allow(dead_code)]
    fn soc_hw_init(&mut self) -> Result<(), SdioError> {
        // TODO: 实现 SoC 级硬件初始化
        // 需要传入正确的硬件配置和延时函数
        Ok(())
    }

    /// SDHCI 控制器软件复位
    fn controller_reset(&self) -> Result<(), SdioError> {
        self.write::<u8>(SDHCI_SOFTWARE_RESET, SWRST_ALL);
        self.wait_reset_complete()
    }

    /// 设置卡检测覆写（WiFi 模块无物理 CD 引脚）
    fn setup_card_detect(&self) -> Result<(), SdioError> {
        // WiFi 模块无物理 CD 引脚, 通过 HOST_CTL1 强制 CARD_INSERTED
        // bit7: CARD_DET_SEL = 1 (使用 CARD_DET_TEST 而非 SD_CD 引脚)
        // bit6: CARD_DET_TEST = 1 (卡已插入)
        let hc = self.read::<u8>(SDHCI_HOST_CONTROL);
        self.write::<u8>(SDHCI_HOST_CONTROL, hc | HC_CARD_DET_TEST | HC_CARD_DET_SEL);
        Ok(())
    }

    /// 上电 3.3V（必须在启动时钟之前）
    fn power_on(&self) -> Result<(), SdioError> {
        self.write::<u8>(SDHCI_POWER_CONTROL, POWER_330V_ON);
        Ok(())
    }

    /// 设置初始低速时钟 400KHz（SD 规范：初始化阶段 ≤ 400KHz）
    fn setup_initial_clock(&self) -> Result<(), SdioError> {
        self.set_clock(400_000)
    }

    /// 使能中断状态位 + 信号（IRQ 驱动模式）
    fn enable_interrupts_irq(&self) -> Result<(), SdioError> {
        irq::irq_state_init(self.base);
        self.write::<u16>(SDHCI_NORM_INT_STS_EN, NORM_INT_ENABLE_MASK);
        self.write::<u16>(SDHCI_ERR_INT_STS_EN, ERR_INT_ENABLE_MASK);
        // IRQ 驱动模式: 使能硬件中断信号
        irq::enable_irq_signals();
        Ok(())
    }
}

impl SdioHost for CviSdhci {
    fn init(&mut self) -> Result<(), SdioError> {
        const OCR_IO_FUNC_SHIFT: u32 = 28;
        const OCR_IO_FUNC_MASK: u32 = 0x7 << OCR_IO_FUNC_SHIFT;

        // Step 1: SDHCI 控制器软件复位
        self.controller_reset()?;

        // Step 2: 设置卡检测覆写（WiFi 模块无物理 CD 引脚）
        self.setup_card_detect()?;

        // Step 3: 上电 3.3V
        self.power_on()?;

        // Step 4: 设置初始低速时钟 400KHz
        self.setup_initial_clock()?;

        // Step 5: 使能中断状态位 + 信号（IRQ 驱动模式）
        // PLIC ISR 已在 aic8800_wireless::connect() 中注册
        self.enable_interrupts_irq()?;

        // Step 6: CMD5 探测 SDIO 卡
        let ocr_query = self.send_cmd(5, 0x0000_0000).inspect_err(|_| {
            log::warn!("[SDIO] CMD5 failed: no SDIO card detected");
        })?;
        let num_io_funcs = ((ocr_query & OCR_IO_FUNC_MASK) >> OCR_IO_FUNC_SHIFT) as u8;
        log::debug!(
            "[SDIO] CMD5: {} IO function(s), memory={}",
            num_io_funcs,
            (ocr_query & OCR_MEM_PRESENT) != 0
        );

        // 选择电压并轮询直到就绪
        let voltage = ocr_query & OCR_VOLTAGE_MASK & OCR_3V2_3V4;
        if voltage == 0 {
            log::error!("[SDIO] No common voltage range");
            return Err(SdioError::Unsupported);
        }
        let mut ready = false;
        for _ in 0..CMD5_OCR_RETRY {
            let resp = self.send_cmd(5, voltage)?;
            if resp & R4_READY != 0 {
                ready = true;
                break;
            }
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
        }
        if !ready {
            log::error!("[SDIO] Card not ready after CMD5 polling");
            return Err(SdioError::Timeout);
        }
        log::debug!("[SDIO] Card ready (IORDY)");

        // Step 7: CMD3 获取 RCA
        let resp = self.send_cmd(3, 0)?;
        self.rca = (resp >> 16) as u16;
        log::debug!("[SDIO] RCA = 0x{:04x}", self.rca);

        // Step 8: CMD7 选卡
        self.send_cmd(7, (self.rca as u32) << 16)?;

        // Step 9: 高速模式
        let bus_speed = self.cmd52_read(0, CCCR_BUS_SPEED_SELECT)?;
        if (bus_speed & 0x01) != 0 {
            self.cmd52_write(0, CCCR_BUS_SPEED_SELECT, bus_speed | 0x02)?;
            let hc1 = self.read::<u8>(SDHCI_HOST_CONTROL);
            self.write::<u8>(SDHCI_HOST_CONTROL, hc1 | HC_HIGH_SPEED);
            self.set_clock(50_000_000)?;
            log::debug!("[SDIO] High-Speed 50MHz enabled");
        } else {
            self.set_clock(25_000_000)?;
        }

        // Step 10: 4-bit 总线宽度
        let bus_if = self.cmd52_read(0, CCCR_BUS_INTERFACE)?;
        self.cmd52_write(0, CCCR_BUS_INTERFACE, (bus_if & 0xFC) | 0x02)?;
        let hc1 = self.read::<u8>(SDHCI_HOST_CONTROL);
        self.write::<u8>(SDHCI_HOST_CONTROL, hc1 | HC_BUS_WIDTH_4);

        // Step 11: 使能 Function 1 并设置块大小
        self.enable_func(1)?;
        self.set_block_size(1, SDIO_DEFAULT_BLOCK_SIZE)?;

        // Step 12: 读取 vendor/device ID
        let (vid, did) = self
            .read_manfid_from_cis(1)
            .or_else(|_| self.read_manfid_from_cis(0))?;
        self.vendor_id = vid;
        self.device_id = did;
        log::debug!("[SDIO] card: vendor=0x{:04x}, device=0x{:04x}", vid, did);

        log::debug!("[SDIO] SDHCI init complete");
        Ok(())
    }

    /// 获取 MMIO 基地址（ISR 等需要直接访问寄存器的场景）
    fn mmio_base(&self) -> usize {
        self.base
    }

    fn read_byte(&self, func: u8, addr: u32) -> Result<u8, SdioError> {
        self.cmd52_read(func, addr)
    }

    fn write_byte(&self, func: u8, addr: u32, val: u8) -> Result<(), SdioError> {
        self.cmd52_write(func, addr, val)
    }

    fn write_byte_read(&self, func: u8, addr: u32, val: u8) -> Result<u8, SdioError> {
        self.cmd52(func, addr, CMD52_RW_FLAG | CMD52_RAW_FLAG, val)
    }

    fn read_fifo(&self, func: u8, addr: u32, buf: &mut [u8]) -> Result<(), SdioError> {
        self.cmd53_read_fixed(func, addr, buf, 512, true)
    }

    fn read_fifo_inc(&self, func: u8, addr: u32, buf: &mut [u8]) -> Result<(), SdioError> {
        let (bs, nb) = self.cmd53_xfer(func, addr, false, true, 512, true, buf.len())?;
        self.pio_read(buf, bs, nb)?;
        self.wait_transfer_complete()
    }

    fn write_fifo(&self, func: u8, addr: u32, buf: &[u8]) -> Result<(), SdioError> {
        self.cmd53_write_fixed(func, addr, buf, 512, true)
    }

    fn write_fifo_inc(&self, func: u8, addr: u32, buf: &[u8]) -> Result<(), SdioError> {
        let (bs, nb) = self.cmd53_xfer(func, addr, true, true, 512, true, buf.len())?;
        self.pio_write(buf, bs, nb)?;
        self.wait_transfer_complete()
    }

    /// 设置指定 SDIO function 的 block size
    ///
    /// Block size 寄存器位置:
    /// - Function 0: CCCR 0x10-0x11
    /// - Function N (1-7): FBR 0x100*N + 0x10-0x11
    ///
    /// 始终通过 function 0 的 CMD52 访问 (CCCR/FBR 地址空间)
    fn set_block_size(&self, func: u8, size: u16) -> Result<(), SdioError> {
        if func > 7 {
            return Err(SdioError::Unsupported);
        }

        // SDIO block size 合法范围: 1-2048, 推荐 2 的幂
        if size == 0 || size > 2048 {
            return Err(SdioError::Unsupported);
        }

        let base = 0x100 * (func as u32);
        // 写低字节
        self.cmd52_write(0, base + 0x10, (size & 0xFF) as u8)?;
        // 写高字节
        self.cmd52_write(0, base + 0x11, ((size >> 8) & 0xFF) as u8)?;
        // 回读验证
        let lo = self.cmd52_read(0, base + 0x10)? as u16;
        let hi = self.cmd52_read(0, base + 0x11)? as u16;
        let readback = (hi << 8) | lo;
        if readback != size {
            return Err(SdioError::IoError);
        }

        Ok(())
    }

    /// 设置 SDIO 总线时钟频率
    ///
    /// # 参数
    /// * `hz` - 目标时钟频率（Hz）
    ///
    /// # 实现步骤
    /// 1. 从 Capabilities 寄存器读取基频
    /// 2. 计算分频值（确保实际频率 ≤ 目标频率）
    /// 3. 停止 SD 时钟输出
    /// 4. 配置并写入分频值
    /// 5. 等待内部时钟稳定
    /// 6. 启用 SD 时钟输出
    ///
    /// # 时钟公式
    /// ```text
    /// 实际频率 = 基频 / (2 × 分频值)
    /// 其中：
    ///   - 基频：来自 Capabilities 寄存器 bits[15:8]，单位 MHz
    ///   - 分频值：10-bit (0-1023)，0 表示不分频
    ///   - 因子 2：SDHCI 规范固定的分频因子
    /// ```
    ///
    /// # 示例
    /// ```ignore
    /// set_clock(400_000)?;    // 设置 400KHz 初始化时钟
    /// set_clock(50_000_000)?;  // 设置 50MHz 高速时钟
    /// ```
    fn set_clock(&self, hz: u32) -> Result<(), SdioError> {
        // ========== Step 1: 读取基频 ==========
        // 从 Capabilities Register (0x40) 读取基频
        // bits[15:8]: Base Clock Frequency (MHz)
        let caps = self.read::<u32>(SDHCI_CAPABILITIES);
        let base_clock_mhz = (caps >> CAPS_BASE_FREQ_SHIFT) & CAPS_BASE_FREQ_MASK;
        let mut base_clock = base_clock_mhz * MHZ_TO_HZ;

        // 如果硬件报告基频为 0，使用 CVI SoC 默认值
        if base_clock == 0 {
            base_clock = FALLBACK_BASE_CLOCK;
        }

        // ========== Step 2: 计算分频值 ==========
        // 目标：确保 base_clock / (2 × divisor) ≤ hz
        // 即：divisor ≥ base_clock / (2 × hz)
        let divisor = if hz >= base_clock {
            // 目标频率 ≥ 基频，不分频
            0u16
        } else {
            // 计算最小分频值（向上取整）
            // 公式：divisor = ceil(base_clock / (2 × hz))
            let div = base_clock.div_ceil(DIV_FACTOR * hz);

            // 限制在 10-bit 分频器范围内 (0-1023)
            div.min(MAX_DIVISOR as u32) as u16
        };

        // ========== Step 3: 停止 SD 时钟输出 ==========
        // 读取当前时钟控制寄存器
        let mut clk_reg = self.read::<u16>(SDHCI_CLOCK_CONTROL);

        // 清除 SD_CLK_EN 和 INT_CLK_EN，停止时钟输出
        clk_reg &= !(CC_SD_CLK_EN | CC_INT_CLK_EN);
        self.write::<u16>(SDHCI_CLOCK_CONTROL, clk_reg);

        // ========== Step 4: 配置分频器 ==========
        // SDHCI 使用 10-bit 分频值：
        //   - 低 8 位写入 bits[15:8] (CC_FREQ_SEL_MASK)
        //   - 高 2 位写入 bits[7:6]   (CC_FREQ_SEL_EXT_MASK)
        clk_reg &= !(CC_FREQ_SEL_MASK | CC_FREQ_SEL_EXT_MASK);

        // 提取分频值的低 8 位，左移 8 位到 bits[15:8]
        let freq_sel = (divisor & DIVISOR_LOW_MASK) << CC_DIV_SHIFT;

        // 提取分频值的高 2 位，左移 6 位到 bits[7:6]
        let ext_sel = ((divisor >> 8) & DIVISOR_HIGH_MASK) << CC_EXT_DIV_SHIFT;

        // 写入分频值并使能内部时钟
        clk_reg |= freq_sel | ext_sel | CC_INT_CLK_EN;
        self.write::<u16>(SDHCI_CLOCK_CONTROL, clk_reg);

        // ========== Step 5: 等待内部时钟稳定 ==========
        self.wait_clock_stable()?;

        // ========== Step 6: 启用 SD 时钟输出 ==========
        clk_reg = self.read::<u16>(SDHCI_CLOCK_CONTROL);
        self.write::<u16>(SDHCI_CLOCK_CONTROL, clk_reg | CC_SD_CLK_EN);

        Ok(())
    }

    /// 使能指定 SDIO function (1-7)  
    ///  
    /// 写 CCCR IO_ENABLE (0x02) 对应位，然后轮询 IO_READY (0x03) 等待就绪  
    fn enable_func(&self, func: u8) -> Result<(), SdioError> {
        if func == 0 || func > 7 {
            return Err(SdioError::Unsupported);
        }

        // 使能对应 function 位
        let io_en = self.cmd52_read(0, CCCR_IO_ENABLE)?;
        self.cmd52_write(0, CCCR_IO_ENABLE, io_en | (1 << func))?;

        // 轮询等待 IO_READY 位被设置
        for _ in 0..1000u32 {
            let io_ready = self.cmd52_read(0, CCCR_IO_READY)?;
            if io_ready & (1 << func) != 0 {
                return Ok(());
            }
            for _ in 0..100_000 {
                core::hint::spin_loop();
            }
        }

        log::error!("SDIO: Function {} not ready after enabling", func);
        Err(SdioError::Timeout)
    }

    fn vendor_device_id(&self) -> (u16, u16) {
        (self.vendor_id, self.device_id)
    }

    fn enable_irq(&self) {
        irq::enable_irq_signals();
    }

    fn disable_irq(&self) {
        irq::disable_irq_signals();
    }

    fn card_irq_ctrl(&self) -> Option<Arc<dyn SdioCardIrq>> {
        Some(Arc::new(CviCardIrqCtrl::new(self.base)))
    }
}
