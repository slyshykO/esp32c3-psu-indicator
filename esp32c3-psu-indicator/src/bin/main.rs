#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use alloc::boxed::Box;
use bt_hci::controller::ExternalController;
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::spi::{
    Mode as SpiMode,
    master::{Config as SpiConfig, Spi},
};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::ble::controller::BleConnector;
use esp32c3_psu_indicator::wifi_scan_task;
use embedded_hal_bus::spi::ExclusiveDevice;
use mcp25xx::bitrates::clock_16mhz::CNF_500K_BPS;
use mcp25xx::registers::{OperationMode, RXB0CTRL, RXM};
use mcp25xx::{Config as McpConfig, MCP25xx};
use trouble_host::prelude::*;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(1))
            .with_mode(SpiMode::_0),
    )
    .expect("SPI init failed: clock or mode out of range")
    .with_sck(peripherals.GPIO8)
    .with_mosi(peripherals.GPIO10)
    .with_miso(peripherals.GPIO9);

    let cs = Output::new(peripherals.GPIO20, Level::High, OutputConfig::default());
    let spi_dev =
        ExclusiveDevice::new_no_delay(spi, cs).expect("SPI device init failed: CS pin invalid");

    let mut mcp25xx = MCP25xx { spi: spi_dev };
    let mcp_config = McpConfig::default()
        .mode(OperationMode::NormalOperation)
        .bitrate(CNF_500K_BPS)
        .receive_buffer_0(RXB0CTRL::default().with_rxm(RXM::ReceiveAny));
    mcp25xx
        .apply_config(&mcp_config)
        .expect("MCP2515 config failed: check wiring/oscillator/bitrate");
    info!("MCP2515 configured over SPI.");

    let radio_init = Box::leak(Box::new(
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    ));
    let (wifi_controller, _interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(radio_init, peripherals.BT, Default::default())
        .expect("BLE transport init failed: controller not ready");
    let ble_controller = ExternalController::<_, 1>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let _stack = trouble_host::new(ble_controller, &mut resources);

    spawner.spawn(wifi_scan_task(wifi_controller)).ok();

    let mut cnt = 0;
    loop {
        info!("Hello world! #{}", cnt);
        cnt += 1;
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}
