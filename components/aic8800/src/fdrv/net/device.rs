//! AIC8800 Wi-Fi network device implementing `NetDriverOps`.
//!
//! This module bridges the `WifiBus` TX/RX queues to the ArceOS network
//! stack (`ax_net`) via the `NetDriverOps` trait.

use alloc::{boxed::Box, sync::Arc, vec, vec::Vec};
use core::{ptr::NonNull, sync::atomic::Ordering};

use ax_driver_base::{BaseDriverOps, DevError, DevResult, DeviceType};
use ax_driver_net::{EthernetAddress, NetBufPtr, NetDriverOps};

use crate::fdrv::{core::bus::WifiBus, thread::tx};

const MAX_TX_QUEUE_LEN: usize = 128;

/// AIC8800 Wi-Fi network device.
///
/// Wraps a shared `WifiBus` and implements `NetDriverOps` so that
/// `ax_net`'s `EthernetDevice` can drive Ethernet-level TX/RX.
pub struct AicWifiNetDev {
    bus: Arc<WifiBus>,
    mac: [u8; 6],
}

// SAFETY: WifiBus internals are protected by atomics and SpinNoIrq locks.
unsafe impl Send for AicWifiNetDev {}
unsafe impl Sync for AicWifiNetDev {}

impl AicWifiNetDev {
    pub fn new(bus: Arc<WifiBus>, mac: [u8; 6]) -> Self {
        Self { bus, mac }
    }

    /// Allocate a buffer backed by a `Box<[u8]>`, returning a raw `NonNull`
    fn alloc_buf(size: usize) -> Option<NonNull<u8>> {
        if size == 0 {
            return None;
        }
        let boxed: Box<[u8]> = vec![0u8; size].into_boxed_slice();
        let ptr = Box::into_raw(boxed) as *mut u8;
        NonNull::new(ptr)
    }

    /// Free a buffer that was allocated by `alloc_buf`.
    ///
    /// # Safety
    /// `ptr` must have been returned by `alloc_buf` with the same `size`.
    fn free_buf(ptr: NonNull<u8>, size: usize) {
        unsafe {
            let raw_slice = core::ptr::slice_from_raw_parts_mut(ptr.as_ptr(), size);
            let _ = Box::from_raw(raw_slice);
        }
    }
}

impl BaseDriverOps for AicWifiNetDev {
    fn device_type(&self) -> DeviceType {
        DeviceType::Net
    }

    fn device_name(&self) -> &str {
        "aic8800-wifi"
    }

    // 返回 SDIO1 CARD_INT 的 PLIC IRQ 号 (SG2002: IRQ#38)
    fn irq_num(&self) -> Option<usize> {
        Some(38)
    }
}

impl NetDriverOps for AicWifiNetDev {
    fn mac_address(&self) -> EthernetAddress {
        EthernetAddress(self.mac)
    }

    fn can_transmit(&self) -> bool {
        self.bus.conn.vif_idx.load(Ordering::Acquire) != 0xFF
            && self.bus.tx.pktcnt.load(Ordering::Acquire) < MAX_TX_QUEUE_LEN as u32
    }

    fn can_receive(&self) -> bool {
        !self.bus.rx.data_queue.lock().is_empty()
    }

    fn rx_queue_size(&self) -> usize {
        self.bus.rx.data_queue.lock().len()
    }

    fn tx_queue_size(&self) -> usize {
        self.bus.tx.pktcnt.load(Ordering::Acquire) as usize
    }

    fn alloc_tx_buffer(&mut self, size: usize) -> DevResult<NetBufPtr> {
        let ptr = Self::alloc_buf(size).ok_or(DevError::NoMemory)?;
        // raw_ptr == pkt_ptr; pkt_len == buf_len == size
        Ok(NetBufPtr::new(ptr, ptr, size))
    }

    fn transmit(&mut self, tx_buf: NetBufPtr) -> DevResult {
        // Copy the Ethernet frame out of the NetBufPtr buffer.
        let eth_frame: Vec<u8> = tx_buf.packet().to_vec();
        let buf_ptr = tx_buf.packet().as_ptr() as *mut u8;
        let buf_size = tx_buf.packet_len();

        // Free the TX buffer immediately (we already copied the data).
        // NetBufPtr does NOT implement Drop, so we must free manually.
        if let Some(nn) = NonNull::new(buf_ptr) {
            Self::free_buf(nn, buf_size);
        }
        // Note: tx_buf goes out of scope here — no Drop, so this is fine.

        // Enqueue the Ethernet frame to the Wi-Fi TX queue.
        // The TX thread will construct the HostDesc and send via SDIO.
        tx::enqueue_data_frame(&self.bus, eth_frame).map_err(|_| DevError::Again)?;

        Ok(())
    }

    fn recycle_tx_buffers(&mut self) -> DevResult {
        // TX buffers are freed immediately in transmit(), nothing to do.
        Ok(())
    }

    fn receive(&mut self) -> DevResult<NetBufPtr> {
        let frame = self
            .bus
            .rx
            .data_queue
            .lock()
            .pop_front()
            .ok_or(DevError::Again)?;
        let size = frame.len();
        let ptr = Self::alloc_buf(size).ok_or(DevError::NoMemory)?;

        // Copy the Ethernet frame into the newly allocated buffer.
        unsafe {
            core::ptr::copy_nonoverlapping(frame.as_ptr(), ptr.as_ptr(), size);
        }

        Ok(NetBufPtr::new(ptr, ptr, size))
    }

    fn recycle_rx_buffer(&mut self, rx_buf: NetBufPtr) -> DevResult {
        let buf_ptr = rx_buf.packet().as_ptr() as *mut u8;
        let buf_size = rx_buf.packet_len();
        // NetBufPtr has no Drop — free manually.
        if let Some(nn) = NonNull::new(buf_ptr) {
            Self::free_buf(nn, buf_size);
        }
        Ok(())
    }
}

use ax_sync::Mutex;

/// 全局暂存：WiFi 网络设备（由 StarryOS 上层取出并注册到 ax_net）
static PENDING_NET_DEV: Mutex<Option<Box<dyn NetDriverOps>>> = Mutex::new(None);

/// 创建 WiFi 网络设备并存入全局，等待上层取出注册
///
/// 由于 aic8800_fdrv 位于 StarryOS 工作区外，无法直接依赖 `axdriver`，
/// 因此不能在此调用 `ax_net::init_network()`。
/// 上层通过 `take_wifi_net_device()` 取出设备后自行注册。
pub fn store_wifi_net_device(bus: Arc<WifiBus>, mac: [u8; 6]) {
    let dev = AicWifiNetDev::new(bus, mac);
    let boxed: Box<dyn NetDriverOps> = Box::new(dev);
    *PENDING_NET_DEV.lock() = Some(boxed);
    log::debug!("[aic8800] Wi-Fi net device stored");
}

/// 取出暂存的 WiFi 网络设备（一次性，取后清空）
///
/// 在 StarryOS 工作区内调用，用于包装为 `AxDeviceContainer` 后
/// 调用 `ax_net::init_network()` 完成注册。
pub fn take_wifi_net_device() -> Option<Box<dyn NetDriverOps>> {
    PENDING_NET_DEV.lock().take()
}
