use crate::{robot::{
    display_manager::DisplayManager,
    events::{DisplayEvents, MotorCommand, RobotEvents, RobotRotation},
    motors_manager::MotorsManager,
}, utils::channels::{ActionPub, EventSub}};
use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Instant};
use embedded_graphics::{pixelcolor::Rgb565, prelude::RgbColor};

pub struct EventLoop<DM: DisplayManager, MM: MotorsManager> {
    display_manager: DM,
    motors_manager: MM,
    _spawner: Spawner,
    last_signal: &'static Signal<NoopRawMutex, ()>,
    event_sub: EventSub,
    _action_pub: ActionPub,
    last_time: Instant,
}

impl<DM: DisplayManager, MM: MotorsManager> EventLoop<DM, MM> {
    pub fn new(
        display_manager: DM,
        motors_manager: MM,
        signal: &'static Signal<NoopRawMutex, ()>,
        event_sub: EventSub,
        action_pub: ActionPub,
        spawner: Spawner,
    ) -> Self {
        Self {
            display_manager,
            motors_manager,
            _spawner: spawner,
            last_signal: signal,
            event_sub,
            _action_pub: action_pub,
            last_time: Instant::now(),
        }
    }

    fn stop_robot_after_timeout(&mut self, timeout_secs: Duration) {
        let now = Instant::now();
        if now - self.last_time > timeout_secs {
            self.motors_manager.stop().unwrap();
            self.last_time = now;
        }
    }

    pub async fn run(&mut self) {
        info!("Event loop started");
        loop {
            self.stop_robot_after_timeout(Duration::from_secs(5));
            match self.event_sub.next_message_pure().await {
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
                RobotEvents::Motor(MotorCommand::Move { left: x, right: y }) => {
                    info!("Moving with left: {}, right: {}", x, y);
                    self.last_signal.signal(());
                    self.motors_manager.drive(x, y).unwrap();
                    self.last_time = Instant::now();
                }
                RobotEvents::Motor(MotorCommand::Rotate { direction: RobotRotation::Clockwise(speed) }) => {
                    self.last_signal.signal(());
                    self.motors_manager.rotate_clockwise(speed).unwrap();
                    self.last_time = Instant::now();
                }
                RobotEvents::Motor(MotorCommand::Rotate { direction: RobotRotation::CounterClockwise(speed) }) => {
                    self.last_signal.signal(());
                    self.motors_manager.rotate_counter_clockwise(speed).unwrap();
                    self.last_time = Instant::now();
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
