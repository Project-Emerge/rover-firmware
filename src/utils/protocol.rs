#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Move {
    pub left: f32,
    pub right: f32,
}

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Battery {
    pub battery_voltage: f32,
    pub battery_percentage: u8,
    pub absorbed_current: i64,
}
