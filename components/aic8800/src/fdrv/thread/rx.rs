use alloc::{sync::Arc, vec, vec::Vec};
use core::{
    future::poll_fn,
    sync::atomic::{AtomicU64, Ordering},
    task::Poll,
};

use ax_task::future::block_on;
use log;

use crate::{
    common::{SDIO_TYPE_CFG, SDIO_TYPE_CFG_CMD_RSP, SDIO_TYPE_CFG_DATA_CFM, SDIO_TYPE_CFG_PRINT},
    fdrv::{
        consts::{
            ETH_P_PAE, MAX_PKT_LEN, RX_ALIGNMENT, RX_HWHRD_LEN, SDIO_OTHER_INTERRUPT,
            SDIOWIFI_FUNC_BLOCKSIZE,
        },
        core::bus::{BusState, WifiBus},
    },
};

pub static RX_WAKE_COUNT: AtomicU64 = AtomicU64::new(0);

fn align_up(val: usize, align: usize) -> usize {
    (val + align - 1) & !(align - 1)
}

/// 启动 wifi-rx 线程
pub fn start(bus: Arc<WifiBus>) {
    ax_task::spawn_with_name(
        move || {
            log::debug!("[wifi-rx] thread started");

            block_on(poll_fn(move |cx| {
                // 检查总线状态
                if *bus.state.lock() == BusState::Down {
                    return Poll::Ready(());
                }

                // 检查并清除 ISR 标志
                if bus.rx.irq_pending.swap(false, Ordering::AcqRel) {
                    RX_WAKE_COUNT.fetch_add(1, Ordering::Relaxed);
                }

                // 处理所有待读数据（内部会 mask CARD_INT，但不 unmask）
                process_rx_frames(&bus);

                // 先注册 waker，再 unmask CARD_INT
                // 这样 ISR 触发时 waker 已经就位，不会丢失唤醒
                bus.rx.irq_pollset.register(cx.waker());

                // 关键：先 register waker，再 unmask CARD_INT
                // 如果 ISR 在 unmask 后立即触发，waker 已经注册好了
                bus.transport.unmask_card_irq();

                // 双重检查：如果 ISR 在 register 和 unmask 之间触发了
                if bus.rx.irq_pending.swap(false, Ordering::AcqRel) {
                    process_rx_frames(&bus);
                    bus.transport.unmask_card_irq();
                    cx.waker().wake_by_ref();
                    return Poll::Pending;
                }

                Poll::Pending
            }))
        },
        "wifi-rx".into(),
    );
}

// ===== RX 帧读取处理 =====

/// 读取 block_cnt 并处理 SDIO_OTHER_INTERRUPT 重试
///
/// # 返回
/// (block_cnt, should_continue) - 读取的块计数和是否应该继续处理
fn read_block_count_with_retry(bus: &WifiBus, other_int_retries: &mut u32) -> (u8, bool) {
    let block_cnt = {
        match bus.transport.read_byte(1, bus.transport.block_cnt_reg()) {
            Ok(v) => v,
            Err(e) => {
                log::error!("[wifi-rx] read block_cnt failed: {:?}", e);
                return (0, false);
            }
        }
    };

    if block_cnt & SDIO_OTHER_INTERRUPT != 0 {
        *other_int_retries += 1;
        if *other_int_retries > 3 {
            log::trace!(
                "[wifi-rx] SDIO_OTHER_INTERRUPT persists after {} retries, giving up",
                other_int_retries
            );
            return (0, false);
        }
        log::trace!(
            "[wifi-rx] SDIO_OTHER_INTERRUPT (0x{:02x}), re-read",
            block_cnt
        );
        return (0, true); // 继续重试
    }

    (block_cnt, true)
}

/// 读取 FIFO 数据
fn read_fifo_data(bus: &WifiBus, block_cnt: u8) -> Option<Vec<u8>> {
    let data_len = (block_cnt as usize) * SDIOWIFI_FUNC_BLOCKSIZE;
    let mut buf = vec![0u8; data_len];

    if let Err(e) = bus
        .transport
        .read_fifo(1, bus.transport.rd_fifo_addr(), &mut buf)
    {
        log::error!("[wifi-rx] read_fifo failed: {:?}", e);
        return None;
    }

    Some(buf)
}

/// 读取 SDIO FIFO 中的所有帧并按类型分发
///
/// 注意：调用方负责在适当时候 unmask CARD_INT（不在本函数内 unmask），
/// 以避免 unmask 和 waker 注册之间的竞态窗口。
fn process_rx_frames(bus: &WifiBus) {
    // ISR 只设 flag 不 mask，这里先 mask CARD_INT 防止重入
    bus.transport.mask_card_irq();

    let mut other_int_retries = 0u32;

    loop {
        // 在轮询循环中也检查 rx_irq_pending
        if bus.rx.irq_pending.swap(false, Ordering::AcqRel) {
            // ISR 触发了，继续读取（不 break）
        }

        let (block_cnt, should_continue) = read_block_count_with_retry(bus, &mut other_int_retries);
        if !should_continue {
            break;
        }

        if block_cnt == 0 {
            break;
        }

        log::trace!("[wifi-rx] block_cnt=0x{:02x}, reading FIFO", block_cnt);

        other_int_retries = 0;

        // 读取 FIFO 数据
        let Some(buf) = read_fifo_data(bus, block_cnt) else {
            break;
        };

        dispatch_frames(bus, &buf);
    }
}

// ===== DATA 帧处理辅助函数 =====

/// 硬件接收头部信息
struct HwRxHdrInfo {
    decr_status: u8,
    is_80211_npdu: bool,
}

/// 802.11 地址信息
struct AddrInfo<'a> {
    da: &'a [u8],
    sa: &'a [u8],
}

/// 从硬件接收头部提取信息
fn extract_hw_rxhdr_info(data_payload: &[u8]) -> HwRxHdrInfo {
    const HWVECT_STATUS_OFFSET: usize = 36;
    const FLAGS_OFFSET: usize = 48;
    const DECR_UNENC: u8 = 0;

    let decr_status = if data_payload.len() > HWVECT_STATUS_OFFSET {
        (data_payload[HWVECT_STATUS_OFFSET] >> 2) & 0x07
    } else {
        DECR_UNENC
    };

    let flags_byte0 = if data_payload.len() > FLAGS_OFFSET {
        data_payload[FLAGS_OFFSET]
    } else {
        0
    };
    let is_80211_npdu = (flags_byte0 >> 1) & 0x01 != 0;

    HwRxHdrInfo {
        decr_status,
        is_80211_npdu,
    }
}

/// 检查是否为 802.11 数据帧
fn is_80211_data_frame(fc0: u8) -> bool {
    (fc0 & 0x0C) == 0x08
}

/// 获取 802.11 头部长度
fn get_80211_header_len(fc0: u8, fc1: u8) -> usize {
    let is_qos = (fc0 & 0x80) != 0;
    let mut hdr_len: usize = if is_qos { 26 } else { 24 };
    if (fc1 & 0x80) != 0 {
        hdr_len += 4; // +HTC
    }
    hdr_len
}

/// 解析 802.11 地址信息
fn parse_80211_addrs(mpdu: &[u8], fc1: u8, pkt_len: usize) -> Option<AddrInfo<'_>> {
    let to_ds = fc1 & 0x01;
    let from_ds = (fc1 >> 1) & 0x01;

    let (da, sa): (&[u8], &[u8]) = match (to_ds, from_ds) {
        (0, 0) => (&mpdu[4..10], &mpdu[10..16]),
        (1, 0) => (&mpdu[16..22], &mpdu[10..16]),
        (0, 1) => (&mpdu[4..10], &mpdu[16..22]),
        _ => {
            if pkt_len < 30 {
                return None;
            }
            (&mpdu[16..22], &mpdu[24..30])
        }
    };

    Some(AddrInfo { da, sa })
}

/// 获取加密头长度
fn get_crypto_header_len(decr_status: u8) -> usize {
    const DECR_CCMP128: u8 = 3;
    const DECR_CCMP256: u8 = 4;
    const DECR_GCMP128: u8 = 5;
    const DECR_GCMP256: u8 = 6;
    const DECR_TKIP: u8 = 2;
    const DECR_WEP: u8 = 1;
    const DECR_WAPI: u8 = 7;

    match decr_status {
        DECR_CCMP128 | DECR_CCMP256 | DECR_GCMP128 | DECR_GCMP256 => 8,
        DECR_TKIP => 8,
        DECR_WEP => 4,
        DECR_WAPI => 18,
        _ => 0,
    }
}

/// 提取以太网类型
fn extract_ethertype(mpdu: &[u8], ether_type_offset: usize, pkt_len: usize) -> Option<u16> {
    if pkt_len < ether_type_offset + 2 {
        log::trace!(
            "[wifi-rx] MPDU too short for LLC/SNAP: pkt_len={}, need={}",
            pkt_len,
            ether_type_offset + 2
        );
        return None;
    }

    Some(u16::from_be_bytes([
        mpdu[ether_type_offset],
        mpdu[ether_type_offset + 1],
    ]))
}

/// 处理 EAPOL 帧
fn process_eapol_frame(bus: &WifiBus, mpdu: &[u8], payload_start: usize, pkt_len: usize) {
    if pkt_len <= payload_start {
        return;
    }

    let raw_eapol = &mpdu[payload_start..];

    let eapol = if raw_eapol.len() >= 4 {
        let body_len = u16::from_be_bytes([raw_eapol[2], raw_eapol[3]]) as usize;
        let actual_len = 4 + body_len;
        if actual_len <= raw_eapol.len() {
            raw_eapol[..actual_len].to_vec()
        } else {
            raw_eapol.to_vec()
        }
    } else {
        raw_eapol.to_vec()
    };

    let mut queue = bus.rx.eapol_queue.lock();
    queue.push_back(eapol);
    drop(queue);
    bus.rx.eapol_pollset.wake();
}

/// 构造并发送以太网帧
fn build_and_enqueue_eth_frame(
    bus: &WifiBus,
    mpdu: &[u8],
    addr_info: &AddrInfo<'_>,
    ether_type_offset: usize,
    payload_start: usize,
    pkt_len: usize,
) {
    const DATA_RX_QUEUE_MAX: usize = 64;

    if pkt_len <= payload_start {
        return;
    }

    let payload = &mpdu[payload_start..];
    let mut eth_frame = Vec::with_capacity(14 + payload.len());
    eth_frame.extend_from_slice(addr_info.da);
    eth_frame.extend_from_slice(addr_info.sa);
    eth_frame.extend_from_slice(&mpdu[ether_type_offset..ether_type_offset + 2]);
    eth_frame.extend_from_slice(payload);

    let mut queue = bus.rx.data_queue.lock();
    if queue.len() >= DATA_RX_QUEUE_MAX {
        queue.pop_front();
    }
    queue.push_back(eth_frame);
    drop(queue);
    bus.rx.data_pollset.wake();
}

/// 处理单个数据帧
fn process_data_frame(bus: &WifiBus, data_payload: &[u8], pkt_len: usize, _mpdu_offset: usize) {
    const MPDU_OFFSET: usize = 60;

    if pkt_len < 24 || data_payload.len() < MPDU_OFFSET + pkt_len {
        log::warn!("[wifi-rx] DATA frame too short for 802.11 header");
        return;
    }

    let mpdu = &data_payload[MPDU_OFFSET..MPDU_OFFSET + pkt_len];
    let fc0 = mpdu[0];
    let fc1 = mpdu[1];

    if !is_80211_data_frame(fc0) {
        return;
    }

    let hdr_len = get_80211_header_len(fc0, fc1);

    let addr_info = match parse_80211_addrs(mpdu, fc1, pkt_len) {
        Some(info) => info,
        None => return,
    };

    let hw_info = extract_hw_rxhdr_info(data_payload);

    if hw_info.is_80211_npdu {
        return;
    }

    let crypto_hdr_len = get_crypto_header_len(hw_info.decr_status);

    let llc_offset = hdr_len + crypto_hdr_len;
    let ether_type_offset = llc_offset + 6;

    let ethertype = match extract_ethertype(mpdu, ether_type_offset, pkt_len) {
        Some(et) => et,
        None => return,
    };

    let payload_start = llc_offset + 8;

    if ethertype == ETH_P_PAE {
        process_eapol_frame(bus, mpdu, payload_start, pkt_len);
    } else {
        build_and_enqueue_eth_frame(
            bus,
            mpdu,
            &addr_info,
            ether_type_offset,
            payload_start,
            pkt_len,
        );
    }
}

// ===== CFG 帧处理辅助函数 =====

/// 处理 CMD_RSP 类型的 CFG 帧
fn process_cmd_rsp(bus: &WifiBus, msg_data: &[u8]) {
    if msg_data.len() < 8 {
        log::warn!(
            "[wifi-rx] process_cmd_rsp: msg_data too short ({})",
            msg_data.len()
        );
        return;
    }

    let msg_id = u16::from_le_bytes([msg_data[0], msg_data[1]]);

    let expected_cfm = bus.cmd.expected_cfm_id.load(Ordering::Acquire);

    if expected_cfm != 0 && msg_id == expected_cfm {
        log::debug!("[wifi-rx] CFM match: msg_id=0x{:04x} -> rsp_queue", msg_id);
        let mut queue = bus.cmd.rsp_queue.lock();
        queue.push_back(msg_data.to_vec());
        drop(queue);
        bus.cmd.rsp_pollset.wake();
    } else {
        log::debug!(
            "[wifi-rx] indication: msg_id=0x{:04x} (expected_cfm=0x{:04x}) -> ind_queue",
            msg_id,
            expected_cfm
        );
        let mut queue = bus.tx.ind_queue.lock();
        queue.push_back(msg_data.to_vec());
        drop(queue);
        bus.tx.ind_pollset.wake();
    }
}

/// 处理 PRINT 类型的 CFG 帧
fn process_print_frame(msg_data: &[u8]) {
    if let Ok(s) = core::str::from_utf8(msg_data) {
        log::info!("[fw-print] {}", s.trim_end_matches('\0'));
    }
}

/// 处理 CFG 帧
fn process_cfg_frame(bus: &WifiBus, msg_data: &[u8], cfg_subtype: u8) {
    match cfg_subtype {
        SDIO_TYPE_CFG_CMD_RSP => {
            process_cmd_rsp(bus, msg_data);
        }
        SDIO_TYPE_CFG_DATA_CFM => {
            // DATA_CFM 处理（当前为空实现）
        }
        SDIO_TYPE_CFG_PRINT => {
            process_print_frame(msg_data);
        }
        _ => {
            log::warn!(
                "[wifi-rx] unknown frame type=0x{:02x}, len={}",
                cfg_subtype,
                msg_data.len()
            );
        }
    }
}

// ===== 主分发函数 =====

/// 解析 SDIO FIFO 中的聚合帧并按类型分发
fn dispatch_frames(bus: &WifiBus, buf: &[u8]) {
    let mut offset = 0;

    while offset + 4 <= buf.len() {
        let pkt_len = u16::from_le_bytes([buf[offset], buf[offset + 1]]) as usize;
        if pkt_len == 0 || pkt_len > MAX_PKT_LEN as usize {
            break;
        }

        let pkt_type = buf[offset + 2] & 0x7F;
        let is_cfg = (pkt_type & SDIO_TYPE_CFG) == SDIO_TYPE_CFG;

        if !is_cfg {
            // ========== DATA 帧 ==========
            let aggr_len = pkt_len + RX_HWHRD_LEN;
            let advance = align_up(aggr_len, RX_ALIGNMENT);

            if offset + aggr_len > buf.len() {
                log::warn!("[wifi-rx] DATA frame truncated at offset={}", offset);
                break;
            }

            let data_payload = &buf[offset..offset + aggr_len];
            process_data_frame(bus, data_payload, pkt_len, 60);

            offset += advance;
        } else {
            // ========== CFG 帧 ==========
            let msg_start = offset + 4;
            let msg_end = msg_start + pkt_len;

            if msg_end > buf.len() {
                log::warn!("[wifi-rx] CFG frame truncated at offset={}", offset);
                break;
            }

            let msg_data = &buf[msg_start..msg_end];
            let cfg_subtype = pkt_type;

            let advance = align_up(pkt_len, RX_ALIGNMENT) + 4;
            process_cfg_frame(bus, msg_data, cfg_subtype);

            offset += advance;
        }
    }
}
