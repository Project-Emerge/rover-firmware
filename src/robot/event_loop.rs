use crate::robot::{
    display_manager::DisplayManager,
    events::{DisplayEvents, MotorCommand, RobotEvents},
};
use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel, signal::Signal};
use embassy_time::{Duration, Timer};
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};

pub struct EventLoop<'a, DM: DisplayManager> {
    command_receiver: &'a Channel<NoopRawMutex, RobotEvents, 64>,
    display_manager: DM,
    spawner: Spawner,
    last_signal: &'static Signal<NoopRawMutex, ()>,
}

impl<'a, DM: DisplayManager> EventLoop<'a, DM> {
    pub fn new(
        command_receiver: &'a Channel<NoopRawMutex, RobotEvents, 64>,
        display_manager: DM,
        signal: &'static Signal<NoopRawMutex, ()>,
        spawner: Spawner,
    ) -> Self {
        Self {
            command_receiver,
            display_manager,
            spawner,
            last_signal: signal,
        }
    }

    pub async fn run(&mut self) {
        loop {
            match self.command_receiver.receive().await {
                RobotEvents::Display(display_events) => {
                    info!("Display event: {:?}", display_events);
                    match display_events {
                        DisplayEvents::ShowBatteryPercentage { percentage } => {
                            self.display_manager
                                .write_battery_percentage(percentage)
                                .unwrap();
                        }
                        DisplayEvents::ShowBatteryVoltage { voltage } => {
                            self.display_manager.write_battery_voltage(voltage).unwrap()
                        }
                        DisplayEvents::ShowCurrent { current } => {
                            self.display_manager.write_current_value(current).unwrap()
                        }
                        DisplayEvents::ShowIp { ip } => {
                            self.display_manager.show_wifi_status(ip).unwrap();
                        }
                        DisplayEvents::ShowIsLeader { is_leader: _ } => todo!(),
                        DisplayEvents::Render => {
                            self.display_manager.clear_display(Rgb565::BLACK).unwrap();
                            self.display_manager.show().unwrap();
                        }
                        DisplayEvents::ShowMemoryStats {
                            free_memory,
                            total_memory,
                        } => self
                            .display_manager
                            .write_memory_stats(free_memory, total_memory)
                            .unwrap(),
                    }
                }
                RobotEvents::Motor(motor_command) => {
                    self.last_signal.signal(());
                    self.spawner
                        .spawn(motor_task(motor_command, self.last_signal))
                        .unwrap()
                }
                RobotEvents::OverCurrent => todo!(),
                RobotEvents::ConnectedToMqtt(is_connected) => {
                    self.display_manager
                        .write_mqtt_status(is_connected)
                        .unwrap();
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn motor_task(command: MotorCommand, signal: &'static Signal<NoopRawMutex, ()>) {
    info!("Motor task started: {}", command);

    select(
        async move {
            // Simulate motor operation
            embassy_time::Timer::after(Duration::from_secs(1)).await;
        },
        signal.wait(),
    )
    .await;
    // Stop the motor or perform any cleanup if necessary
    // TODO!
    info!("Motor task completed: {}", command);
}
