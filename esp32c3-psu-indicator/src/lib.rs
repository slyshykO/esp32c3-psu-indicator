#![no_std]

use defmt::info;
use embassy_time::{Duration, Timer};
use esp_radio::wifi::{Config, WifiController, scan::ScanConfig, sta::StationConfig};

#[embassy_executor::task]
pub async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
	// Applying station configuration also starts Wi-Fi in the new radio API.
	let mode_config = Config::Station(StationConfig::default());
	if let Err(err) = wifi_controller.set_config(&mode_config) {
		info!("Wi-Fi config failed: {:?}", err);
		return;
	}

	loop {
		info!("Starting Wi-Fi scan...");
		match wifi_controller
			.scan_async(&ScanConfig::default())
			.await
		{
			Ok(aps) => {
				info!("Found {} networks", aps.len());
				for ap in aps {
					info!(
						"ssid={:?} rssi={} ch={} auth={:?}",
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
