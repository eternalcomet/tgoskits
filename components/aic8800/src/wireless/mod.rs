//! AIC8800 WiFi 顶层接口
//!
//! 元 crate：封装从 SDHCI 初始化到网络注册的完整流程。
//! 应用层只需调用一个函数即可使用 WiFi。
//!
//! # 使用示例
//!
//! ```no_run
//! let bus =
//!     aic8800_wireless::connect("MyWiFi", "password123", "192.168.1.200", 24, "192.168.1.1")
//!         .expect("WiFi connect failed");
//!
//! // ... 使用网络 ...
//!
//! aic8800_wireless::shutdown(&bus);
//! ```

use alloc::sync::Arc;

#[cfg(feature = "speed-test")]
pub mod speed_test;

use ax_plat_riscv64_sg2002::config::{devices::*, plat::PHYS_VIRT_OFFSET};
use sdhci_cv1800::{
    CviSdhci,
    hw_init::{Sdio1HwConfig, sdio1_hw_init},
};
use sdio_host::SdioHost;

pub use crate::fdrv::WifiError;
use crate::{
    common::ChipVariant,
    fdrv::{WifiBus, WifiClient, WifiConfig},
};

pub fn connect(
    ssid: &str,
    password: &str,
    ip: &str,
    prefix: u8,
    gateway: &str,
) -> Result<Arc<WifiBus>, WifiError> {
    let hw_cfg = Sdio1HwConfig::new(
        CRG_PADDR,
        SYSCTRL_PADDR,
        RTCSYS_CTRL_PADDR,
        RTCSYS_IO_PADDR,
        SDIO1_PADDR,
        PHYS_VIRT_OFFSET,
    );
    sdio1_hw_init(&hw_cfg);

    ax_plat_riscv64_sg2002::irq::register_sdio1_irq(sdhci_cv1800::irq::sdhci_irq_handler);

    // ---- Step 3: SDHCI 控制器初始化 ----
    let mut sdio = CviSdhci::new(hw_cfg.sdio1_base_va);
    sdio.init().map_err(|e| {
        log::error!("[aic8800] SDIO init failed: {:?}", e);
        WifiError::NotInitialized
    })?;

    // ---- Step 3: 自动检测芯片型号 ----
    let (vid, did) = sdio.vendor_device_id();
    let chip = ChipVariant::from_vid_did(vid, did);
    log::info!(
        "[aic8800] chip={:?} vid=0x{:04x} did=0x{:04x}",
        chip,
        vid,
        did
    );

    if chip == ChipVariant::Unknown {
        return Err(WifiError::OperationFailed(alloc::format!(
            "Unknown WiFi chip: vid=0x{:04x}, did=0x{:04x}",
            vid,
            did
        )));
    }

    // ---- Step 4: 固件加载 ----
    crate::fw::firmware_init(&mut sdio, chip).map_err(|e| {
        log::error!("[aic8800] Firmware init failed: {:?}", e);
        WifiError::NotInitialized
    })?;
    log::info!("[aic8800] Firmware loaded");

    // ---- Step 5: 驱动初始化（SdioTransport + IRQ + RX/TX 线程） ----
    let bus = crate::fdrv::init(sdio, chip).map_err(|e| {
        log::error!("[aic8800] FDRV init failed: {}", e);
        WifiError::NotInitialized
    })?;

    // ---- Step 5.5: 注册 CARD_INT 回调 ----
    sdhci_cv1800::irq::register_card_irq_callback(crate::fdrv::sdio1_irq_handler);

    // ---- Step 6: LMAC 配置 ----
    let mut client = WifiClient::new(Arc::clone(&bus));
    let mac = client.lmac_configure(chip, 6000).map_err(|e| {
        log::error!("[aic8800] LMAC configure failed: {:?}", e);
        e
    })?;

    // ---- Step 7: 扫描 + 连接 ----
    let config = if password.is_empty() {
        WifiConfig::open(ssid)
    } else {
        WifiConfig::wpa2_psk(ssid, password)
    };
    client.connect(&config, 15000)?;
    log::info!("[aic8800] Connected to '{}' MAC={:02x?}", ssid, mac);

    // ---- Step 8: 注册网络设备到 ax_net ----
    client.store_net_device();

    Ok(bus)
}

/// 关闭 WiFi 连接并释放所有资源
///
/// 执行顺序：
/// 1. 断开 WiFi 连接（发送 disconnect 命令给固件）
/// 2. 关闭总线（停止 TX/RX 线程、清空队列、禁用 SDIO 中断）
/// 3. 释放全局 bus 引用
pub fn shutdown(bus: &Arc<WifiBus>) {
    let client = WifiClient::new(Arc::clone(bus));
    if let Err(e) = client.disconnect() {
        log::warn!("[aic8800] Disconnect error: {:?}", e);
    }

    bus.shutdown();

    log::info!("[aic8800] Shutdown complete");
}
