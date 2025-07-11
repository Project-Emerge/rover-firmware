use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use esp_hal::{i2c::master::I2c, Async};
use defmt::info;
use ina219::{
    address::Address,
    calibration::{IntCalibration, MicroAmpere},
    SyncIna219,
};

use crate::robot::events::RobotEvents;

pub async fn monitor_battery_loop(
    i2c: I2c<'static, Async>,
    command_sender: &Channel<NoopRawMutex, RobotEvents<'static>, 20>,
) -> Result<(), ()> {
    // Create INA219 instance
    info!("INA219 initialized successfully");
    let calibration = IntCalibration::new(MicroAmpere(1_000_000), 1_000).unwrap();
    let mut ina =
        SyncIna219::new_calibrated(i2c, Address::from_byte(0x40).unwrap(), calibration).unwrap();

    loop {
        let conversion_time = ina.configuration().unwrap().conversion_time_us().unwrap();
        Timer::after(Duration::from_micros(conversion_time as u64)).await;
        // Read the next measurement
        let measurement = ina
            .next_measurement()
            .unwrap()
            .expect("Conversion is done now");
        let absorbed_current = measurement.current.0 / 100_000;
        let battery_voltage = measurement.bus_voltage.voltage_mv() as f32 / 1000.0;

        info!(
            "Current: {} mA, Battery Voltage: {} V",
            absorbed_current,
            battery_voltage
        );

        command_sender.send(RobotEvents::Display(
            crate::robot::events::DisplayEvents::ShowCurrent { current: absorbed_current },
        )).await;
        command_sender.send(RobotEvents::Display(
            crate::robot::events::DisplayEvents::ShowBatteryVoltage { voltage: battery_voltage },
        )).await;

        Timer::after(Duration::from_secs(1)).await;
    }
}
