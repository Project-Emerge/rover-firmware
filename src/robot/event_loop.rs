use crate::robot::{
    display_manager::DisplayManager,
    events::{DisplayEvents, MotorCommand, RobotEvents, RobotRotation},
    motors_manager::MotorsManager,
};
use defmt::{debug, info};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel, signal::Signal};
use embassy_time::Duration;
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};

pub struct EventLoop<DM: DisplayManager, MM: MotorsManager> {
    command_receiver: &'static Channel<NoopRawMutex, RobotEvents, 64>,
    display_manager: DM,
    motors_manager: MM,
    spawner: Spawner,
    last_signal: &'static Signal<NoopRawMutex, ()>,
}

impl<DM: DisplayManager, MM: MotorsManager> EventLoop<DM, MM> {
    pub fn new(
        command_receiver: &'static Channel<NoopRawMutex, RobotEvents, 64>,
        display_manager: DM,
        motors_manager: MM,
        signal: &'static Signal<NoopRawMutex, ()>,
        spawner: Spawner,
    ) -> Self {
        Self {
            command_receiver,
            display_manager,
            motors_manager,
            spawner,
            last_signal: signal,
        }
    }

    pub async fn run(&mut self) {
        self.spawner
            .spawn(motor_timeout_task(self.command_receiver, self.last_signal))
            .unwrap();
        info!("Event loop started");
        loop {
            match self.command_receiver.receive().await {
                RobotEvents::Display(DisplayEvents::ShowBatteryPercentage { percentage }) => {
                    self.display_manager
                        .write_battery_percentage(percentage)
                        .unwrap();
                }
                RobotEvents::Display(DisplayEvents::ShowBatteryVoltage { voltage }) => {
                    self.display_manager.write_battery_voltage(voltage).unwrap()
                }
                RobotEvents::Display(DisplayEvents::ShowCurrent { current }) => {
                    self.display_manager.write_current_value(current).unwrap()
                }
                RobotEvents::Display(DisplayEvents::ShowIp { ip }) => {
                    self.display_manager.show_wifi_status(ip).unwrap();
                }
                RobotEvents::Display(DisplayEvents::ShowIsLeader { is_leader: _ }) => todo!(),
                RobotEvents::Display(DisplayEvents::Render) => {
                    self.display_manager.clear_display(Rgb565::BLACK).unwrap();
                    self.display_manager.show().unwrap();
                }
                RobotEvents::Display(DisplayEvents::ShowMemoryStats {
                    free_memory,
                    total_memory,
                }) => self
                    .display_manager
                    .write_memory_stats(free_memory, total_memory)
                    .unwrap(),
                RobotEvents::Motor(MotorCommand::Move { x, y }) => {
                    self.last_signal.signal(());
                    self.motors_manager.drive(x, y).unwrap();
                }
                RobotEvents::Motor(MotorCommand::Rotate { direction: RobotRotation::Clockwise(speed) }) => {
                    self.last_signal.signal(());
                    self.motors_manager.rotate_clockwise(speed).unwrap();
                }
                RobotEvents::Motor(MotorCommand::Rotate { direction: RobotRotation::CounterClockwise(speed) }) => {
                    self.last_signal.signal(());
                    self.motors_manager.rotate_counter_clockwise(speed).unwrap();
                }
                RobotEvents::Motor(MotorCommand::StopMotors) => {
                    self.last_signal.signal(());
                    self.motors_manager.stop().unwrap();
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
async fn motor_timeout_task(
    event_channel: &'static Channel<NoopRawMutex, RobotEvents, 64>,
    signal: &'static Signal<NoopRawMutex, ()>,
) {
    loop {
        let res = select(
            async move {
                embassy_time::Timer::after(Duration::from_secs(5)).await;
                event_channel
                    .send(RobotEvents::Motor(MotorCommand::StopMotors))
                    .await;
            },
            signal.wait(),
        )
        .await;

        match res {
            Either::First(_) => debug!("Motor task completed successfully"),
            Either::Second(_) => debug!("Motor task interrupted by signal"),
        }
    }
}
