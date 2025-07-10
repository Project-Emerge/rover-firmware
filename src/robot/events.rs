#[derive(Debug, Clone, defmt::Format)]
pub enum RobotEvents<'a> {
    // Display events
    Display(DisplayEvents<'a>),

    // Motor Commands
    Motor(MotorCommand),

    OverCurrent,
}

#[derive(Debug, Clone, defmt::Format)]
pub enum DisplayEvents<'a> {
    ShowCurrent { current: i64 },
    ShowIp { ip: &'a str },
    ShowBatteryPercentage { percentage: u8 },
    ShowBatteryVoltage { voltage: f32 },
    ShowIsLeader { is_leader: bool },
    Render
}

#[derive(Debug, Clone, defmt::Format)]
pub enum RobotRotation {
    Clockwise,
    CounterClockwise,
}

#[derive(Debug, Clone, defmt::Format)]
pub enum MotorCommand {
    MovementVector { x: i64, y: i64 },
    Rotate { direction: RobotRotation }, // Speed in percentage
    SetMotorSpeed { speed: u8 },
    SetMovementTime { time: f32 }, // Time in seconds
    StopMotors,
}