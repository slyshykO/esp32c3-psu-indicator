#![no_std]

use defmt::info;
use embassy_time::{Duration, Timer};
use esp_radio::wifi::{ClientConfig, ModeConfig, ScanConfig, WifiController};

#[embassy_executor::task]
pub async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
	let mode_config = ModeConfig::Client(ClientConfig::default());
	if let Err(err) = wifi_controller.set_config(&mode_config) {
		info!("Wi-Fi config failed: {:?}", err);
		return;
	}

	if let Err(err) = wifi_controller.start_async().await {
		info!("Wi-Fi start failed: {:?}", err);
		return;
	}

	loop {
		info!("Starting Wi-Fi scan...");
		match wifi_controller
			.scan_with_config_async(ScanConfig::default())
			.await
		{
			Ok(aps) => {
				info!("Found {} networks", aps.len());
				for ap in aps {
					info!(
						"ssid={=str} rssi={} ch={} auth={:?}",
						ap.ssid,
						ap.signal_strength,
						ap.channel,
						ap.auth_method
					);
				}
			}
			Err(err) => info!("Wi-Fi scan failed: {:?}", err),
		}

		Timer::after(Duration::from_secs(15)).await;
	}
}
