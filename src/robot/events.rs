#[derive(Debug, Clone, defmt::Format)]
pub enum RobotEvents {
    // Display events
    Display(DisplayEvents),

    // Motor Commands
    Motor(MotorCommand),

    OverCurrent,

    ConnectedToMqtt(bool),
}

#[derive(Debug, Clone, defmt::Format, serde::Serialize, serde::Deserialize)]
pub enum DisplayEvents {
    ShowCurrent { current: i64 },
    ShowIp { ip: heapless::String<64> },
    ShowBatteryPercentage { percentage: u8 },
    ShowBatteryVoltage { voltage: f32 },
    ShowIsLeader { is_leader: bool },
    ShowMemoryStats { free_memory: f32, total_memory: f32 },
    Render
}

#[derive(Debug, Clone, defmt::Format, serde::Serialize, serde::Deserialize)]
pub enum RobotRotation {
    Clockwise(f32),
    CounterClockwise(f32),
}

#[derive(Debug, Clone, defmt::Format)]
pub enum MotorCommand {
    Move { left: f32, right: f32 },
    Rotate { direction: RobotRotation }, // Speed in percentage
    StopMotors,
}
