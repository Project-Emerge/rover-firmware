pub const ROBOT_MOVE_TOPIC: &str = concat!("robots/", env!("ROBOT_ID"), "/move");
pub const ROBOT_BATTERY_TOPIC: &str = concat!("robots/", env!("ROBOT_ID"), "/battery");