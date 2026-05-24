use alloc::{sync::Arc, vec, vec::Vec};
use core::{future::poll_fn, sync::atomic::Ordering, task::Poll};

use ax_task::future::block_on;
use log;

use crate::{
    common::{SDIO_TYPE_DATA, crc8_ponl_107},
    fdrv::{
        consts::{
            DATA_FLOW_CTRL_THRESH, MAX_TX_QUEUE_LEN, SDIOWIFI_FUNC_BLOCKSIZE, TAIL_LEN,
            TX_ALIGNMENT, TX_BATCH_LIMIT,
        },
        core::bus::{BusState, TxFrame, WifiBus},
    },
};

#[derive(Debug)]
pub enum TxError {
    QueueFull,
}

fn align_up(val: usize, align: usize) -> usize {
    (val + align - 1) & !(align - 1)
}

fn has_pending_work(bus: &WifiBus) -> bool {
    bus.cmd.pending_flag.load(Ordering::Acquire) || bus.tx.pktcnt.load(Ordering::Acquire) > 0
}

/// 对 CMD 帧做 TX_ALIGNMENT + TAIL_LEN + BLOCK_SIZE 对齐
fn pad_cmd_frame(cmd: &mut Vec<u8>) -> usize {
    let aligned = align_up(cmd.len(), TX_ALIGNMENT);
    cmd.resize(aligned, 0);

    if cmd.len() % SDIOWIFI_FUNC_BLOCKSIZE != 0 {
        cmd.extend_from_slice(&[0u8; TAIL_LEN]);
    }

    let final_len = align_up(cmd.len(), SDIOWIFI_FUNC_BLOCKSIZE);
    cmd.resize(final_len, 0);
    final_len
}

/// 启动 wifi-tx 线程
pub fn start(bus: Arc<WifiBus>) {
    ax_task::spawn_with_name(
        move || {
            log::debug!("[wifi-tx] thread started");
            block_on(poll_fn(|cx| {
                // 检查总线状态
                if *bus.state.lock() == BusState::Down {
                    return Poll::Ready(());
                }

                // 处理所有待发帧
                let did_work = tx_process(&bus);

                // 注册 waker
                bus.tx.wake_pollset.register(cx.waker());

                // 双重检查
                let pending = has_pending_work(&bus);
                if did_work || pending {
                    cx.waker().wake_by_ref();
                }

                // 只在有错误时唤醒 rsp_pollset，避免无意义的任务切换
                if bus.cmd.rsp_error.load(Ordering::Acquire) {
                    bus.cmd.rsp_pollset.wake();
                }

                Poll::Pending
            }))
        },
        "wifi-tx".into(),
    );
}

// ===== CMD 发送处理 =====

/// 处理 CMD 帧发送
fn process_cmd_tx(bus: &WifiBus) -> bool {
    if !bus.cmd.pending_flag.load(Ordering::Acquire) {
        return false;
    }

    log::warn!("[wifi-tx] process_cmd_tx: found pending CMD");

    let cmd_buf = bus.cmd.pending.lock().take();
    let Some(mut cmd) = cmd_buf else {
        return false;
    };

    bus.cmd.pending_flag.store(false, Ordering::Release);

    log::warn!(
        "[wifi-tx] CMD frame header: [{:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} \
         {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}]",
        cmd.get(0).unwrap_or(&0),
        cmd.get(1).unwrap_or(&0),
        cmd.get(2).unwrap_or(&0),
        cmd.get(3).unwrap_or(&0),
        cmd.get(4).unwrap_or(&0),
        cmd.get(5).unwrap_or(&0),
        cmd.get(6).unwrap_or(&0),
        cmd.get(7).unwrap_or(&0),
        cmd.get(8).unwrap_or(&0),
        cmd.get(9).unwrap_or(&0),
        cmd.get(10).unwrap_or(&0),
        cmd.get(11).unwrap_or(&0),
        cmd.get(12).unwrap_or(&0),
        cmd.get(13).unwrap_or(&0),
        cmd.get(14).unwrap_or(&0),
        cmd.get(15).unwrap_or(&0),
    );

    let send_len = pad_cmd_frame(&mut cmd);

    let transport = &bus.transport;
    transport.mask_card_irq();

    let (fc_ok, did_work) = perform_cmd_flow_control_and_send(transport, &cmd, send_len);

    if fc_ok && did_work {
        log::debug!("[wifi-tx] process_cmd_tx: CMD sent OK, len={}", send_len);
        bus.rx.irq_pollset.wake();
        transport.unmask_card_irq();
    } else if !fc_ok {
        log::error!("[wifi-tx] CMD flow_ctrl timeout, dropping CMD");
        bus.cmd.rsp_error.store(true, Ordering::Release);
        bus.cmd.rsp_pollset.wake();
        transport.unmask_card_irq();
    } else {
        transport.unmask_card_irq();
    }

    did_work
}

/// 执行 CMD 流控检查并发送
fn perform_cmd_flow_control_and_send(
    transport: &crate::fdrv::core::sdio_transport::SdioTransport,
    cmd: &[u8],
    send_len: usize,
) -> (bool, bool) {
    let fc_ok = transport.wait_flow_ctrl_for_size(send_len, 10);

    let did_work = if fc_ok {
        match transport.write_fifo(1, transport.wr_fifo_addr(), cmd) {
            Ok(()) => true,
            Err(e) => {
                log::error!("[wifi-tx] CMD write_fifo failed: {:?}", e);
                false
            }
        }
    } else {
        false
    };

    (fc_ok, did_work)
}

// ===== 数据发送处理 =====

/// 处理数据帧批量发送
fn process_data_tx(bus: &WifiBus) -> bool {
    let vif_idx = bus.conn.vif_idx.load(Ordering::Acquire);
    let sta_idx = bus.conn.sta_idx.load(Ordering::Acquire);

    // 未连接则清空队列
    if vif_idx == 0xFF {
        while let Some(_) = bus.tx.queue.lock().pop_front() {
            bus.tx.pktcnt.fetch_sub(1, Ordering::AcqRel);
        }
        return false;
    }

    let pktcnt = bus.tx.pktcnt.load(Ordering::Acquire);
    if pktcnt == 0 {
        return false;
    }

    let mut did_work = false;
    let mut batch_count: u32 = 0;

    while bus.tx.pktcnt.load(Ordering::Acquire) > 0 {
        if batch_count >= TX_BATCH_LIMIT {
            break;
        }

        if bus.cmd.pending_flag.load(Ordering::Acquire) {
            break;
        }

        if *bus.state.lock() == BusState::Down {
            break;
        }

        if !check_data_flow_control(&bus.transport) {
            break;
        }

        if send_single_data_frame(&bus.transport, bus, vif_idx, sta_idx) {
            did_work = true;
            batch_count += 1;
        }
    }

    did_work
}

/// 检查数据流控状态（带重试）
fn check_data_flow_control(transport: &crate::fdrv::core::sdio_transport::SdioTransport) -> bool {
    for _ in 0..50 {
        match transport.read_flow_ctrl_value() {
            Ok(fc) if fc > DATA_FLOW_CTRL_THRESH => return true,
            Ok(fc) => {
                log::debug!("[wifi-tx] data flow ctrl low: fc={}", fc);
            }
            Err(_) => {}
        }
        ax_task::yield_now();
    }
    log::warn!("[wifi-tx] data flow ctrl timeout");
    false
}

/// 发送单个数据帧
fn send_single_data_frame(
    transport: &crate::fdrv::core::sdio_transport::SdioTransport,
    bus: &WifiBus,
    vif_idx: u8,
    sta_idx: u8,
) -> bool {
    const ETH_HEADER_LEN: usize = 14;

    let frame = bus.tx.queue.lock().pop_front();
    let Some(frame) = frame else {
        return false;
    };
    bus.tx.pktcnt.fetch_sub(1, Ordering::AcqRel);

    let eth_frame = &frame.data;
    if eth_frame.len() < ETH_HEADER_LEN {
        return false;
    }

    let buf = match build_data_frame(eth_frame, vif_idx, sta_idx, transport.is_v3()) {
        Ok(b) => b,
        Err(_) => return false,
    };

    if let Err(e) = transport.write_fifo(1, transport.wr_fifo_addr(), &buf) {
        log::error!("[wifi-tx] DATA write_fifo failed: {:?}", e);
        return false;
    }

    true
}

/// 数据帧构造错误
enum DataFrameBuildError {
    InvalidFrameLength,
}

/// 构造 SDIO 数据帧
fn build_data_frame(
    eth_frame: &[u8],
    vif_idx: u8,
    sta_idx: u8,
    is_v3: bool,
) -> Result<Vec<u8>, DataFrameBuildError> {
    const SDIO_HEADER_LEN: usize = 4;
    const HOSTDESC_SIZE: usize = 28;
    const ETH_HEADER_LEN: usize = 14;

    if eth_frame.len() < ETH_HEADER_LEN {
        return Err(DataFrameBuildError::InvalidFrameLength);
    }

    let eth_dest = &eth_frame[0..6];
    let eth_src = &eth_frame[6..12];
    let ethertype = &eth_frame[12..14];
    let payload = &eth_frame[ETH_HEADER_LEN..];
    let payload_len = payload.len();

    let sdio_payload_len = payload_len + HOSTDESC_SIZE;
    let raw_len = sdio_payload_len + SDIO_HEADER_LEN;
    let aligned_len = align_up(raw_len, TX_ALIGNMENT);
    let sdio_hdr_len = aligned_len - SDIO_HEADER_LEN;

    let final_len = if aligned_len % SDIOWIFI_FUNC_BLOCKSIZE != 0 {
        align_up(aligned_len + TAIL_LEN, SDIOWIFI_FUNC_BLOCKSIZE)
    } else {
        aligned_len
    };

    let mut buf = vec![0u8; final_len];

    // 填充 SDIO header
    buf[0] = (sdio_hdr_len & 0xFF) as u8;
    buf[1] = ((sdio_hdr_len >> 8) & 0x0F) as u8;
    buf[2] = SDIO_TYPE_DATA;
    buf[3] = if is_v3 {
        crc8_ponl_107(&buf[0..3])
    } else {
        0x00
    };

    // 填充 HostDesc
    fill_hostdesc(
        &mut buf,
        payload_len,
        eth_dest,
        eth_src,
        ethertype,
        vif_idx,
        sta_idx,
    );

    // 填充 payload
    let payload_start = SDIO_HEADER_LEN + HOSTDESC_SIZE;
    buf[payload_start..payload_start + payload_len].copy_from_slice(payload);

    Ok(buf)
}

/// 填充 HostDesc
fn fill_hostdesc(
    buf: &mut [u8],
    payload_len: usize,
    eth_dest: &[u8],
    eth_src: &[u8],
    ethertype: &[u8],
    vif_idx: u8,
    sta_idx: u8,
) {
    const SDIO_HEADER_LEN: usize = 4;
    const HOSTDESC_SIZE: usize = 28;

    let hd = &mut buf[SDIO_HEADER_LEN..SDIO_HEADER_LEN + HOSTDESC_SIZE];

    hd[0..2].copy_from_slice(&(payload_len as u16).to_le_bytes());
    // flags_ext [2..4] = 0

    // hostid [4..8]: 设置 bit 31 请求 TX CFM
    hd[4..8].copy_from_slice(&0x8000_0001u32.to_le_bytes());

    hd[8..14].copy_from_slice(eth_dest);
    hd[14..20].copy_from_slice(eth_src);
    hd[20..22].copy_from_slice(ethertype);
    hd[22] = 0;

    hd[23] = 0; // tid = BE

    hd[24] = vif_idx;
    hd[25] = sta_idx;
    // flags [26..28] = 0
}

// ===== 主处理函数 =====

/// TX 处理主逻辑
fn tx_process(bus: &WifiBus) -> bool {
    let mut did_work = false;

    if *bus.state.lock() == BusState::Down {
        return false;
    }

    // Step 1: CMD 优先发送
    if process_cmd_tx(bus) {
        did_work = true;
    }

    // Step 2: DATA 批量发送
    if process_data_tx(bus) {
        did_work = true;
    }

    did_work
}

/// 将以太网帧入队 TX 队列
pub fn enqueue_data_frame(bus: &Arc<WifiBus>, eth_frame: Vec<u8>) -> Result<(), TxError> {
    let mut queue = bus.tx.queue.lock();
    if queue.len() >= MAX_TX_QUEUE_LEN {
        return Err(TxError::QueueFull);
    }

    queue.push_back(TxFrame {
        data: eth_frame,
        priority: 0,
    });
    drop(queue);

    bus.tx.pktcnt.fetch_add(1, Ordering::AcqRel);
    bus.tx.wake_pollset.wake();
    Ok(())
}
