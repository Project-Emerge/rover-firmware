use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use esp_hal::{i2c::master::I2c, Async};
use defmt::{info, error};
use ina219::{
    address::Address,
    calibration::{IntCalibration, MicroAmpere},
    SyncIna219,
};

use crate::robot::events::RobotEvents;

pub async fn monitor_battery_loop(
    i2c: I2c<'static, Async>,
    command_sender: &Channel<NoopRawMutex, RobotEvents, 64>,
) -> Result<(), ()> {
    // Create INA219 instance
    info!("INA219 initialized successfully");
    let calibration = IntCalibration::new(MicroAmpere(1_000_000), 1_000).unwrap();
    
    let mut ina = match SyncIna219::new_calibrated(i2c, Address::from_byte(0x40).unwrap(), calibration) {
        Ok(sensor) => sensor,
        Err(_) => {
            error!("Failed to initialize INA219 sensor");
            return Err(());
        }
    };

    loop {
        let conversion_time = match ina.configuration() {
            Ok(config) => config.conversion_time_us().unwrap_or(1000),
            Err(_) => {
                error!("Failed to get INA219 configuration");
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        };
        
        Timer::after(Duration::from_micros(conversion_time as u64)).await;
        
        // Read the next measurement with error handling
        let measurement = match ina.next_measurement() {
            Ok(Some(m)) => m,
            Ok(None) => {
                info!("INA219 conversion not ready, waiting...");
                Timer::after(Duration::from_millis(100)).await;
                continue;
            },
            Err(_) => {
                error!("Failed to read INA219 measurement");
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        };
        
        let absorbed_current = measurement.current.0 / 100_000;
        let battery_voltage = measurement.bus_voltage.voltage_mv() as f32 / 1000.0;

        info!(
            "Current: {} mA, Battery Voltage: {} V",
            absorbed_current,
            battery_voltage
        );

        // Send events with timeout to prevent blocking
        let _ = embassy_time::with_timeout(Duration::from_millis(500), async {
            command_sender.send(RobotEvents::Display(
                crate::robot::events::DisplayEvents::ShowCurrent { current: absorbed_current },
            )).await;
            command_sender.send(RobotEvents::Display(
                crate::robot::events::DisplayEvents::ShowBatteryVoltage { voltage: battery_voltage },
            )).await;
        }).await;

        Timer::after(Duration::from_secs(5)).await;
    }
}
