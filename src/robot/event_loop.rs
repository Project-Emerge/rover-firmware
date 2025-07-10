use crate::robot::{
    display_manager::DisplayManager,
    events::{DisplayEvents, RobotEvents},
};
use defmt::info;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};

pub struct EventLoop<'a, DM: DisplayManager> {
    command_receiver: &'a Channel<NoopRawMutex, RobotEvents<'a>, 20>,
    display_manager: DM,
}

impl<'a, DM: DisplayManager> EventLoop<'a, DM> {
    pub fn new(
        command_receiver: &'a Channel<NoopRawMutex, RobotEvents<'a>, 20>,
        display_manager: DM,
    ) -> Self {
        Self {
            command_receiver,
            display_manager,
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
                        DisplayEvents::ShowIsLeader { is_leader } => todo!(),
                        DisplayEvents::Render => {
                            self.display_manager
                                .clear_display(Rgb565::BLACK)
                                .unwrap();
                            self.display_manager.show().unwrap();
                        }
                    }
                }
                RobotEvents::Motor(motor_command) => todo!(),
                RobotEvents::OverCurrent => todo!(),
            }
        }
    }
}
