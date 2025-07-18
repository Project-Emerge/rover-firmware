use defmt::info;
use mountain_mqtt::{client::{Client, ClientError, EventHandlerError}, mqtt_manager::{ConnectionId, MqttOperations}, packets::publish::ApplicationMessage};
use mountain_mqtt_embassy::mqtt_manager::FromApplicationMessage;

#[derive(Debug, Clone, defmt::Format)]
pub enum RobotEvents {
    // Display events
    Display(DisplayEvents),

    // Motor Commands
    Motor(MotorCommand),

    OverCurrent,

    ConnectedToMqtt(bool),
}

#[derive(Debug, Clone, defmt::Format)]
pub enum DisplayEvents {
    ShowCurrent { current: i64 },
    ShowIp { ip: heapless::String<64> },
    ShowBatteryPercentage { percentage: u8 },
    ShowBatteryVoltage { voltage: f32 },
    ShowIsLeader { is_leader: bool },
    ShowMemoryStats { free_memory: f32, total_memory: f32 },
    Render
}

#[derive(Debug, Clone, defmt::Format)]
pub enum RobotRotation {
    Clockwise(f32),
    CounterClockwise(f32),
}

#[derive(Debug, Clone, defmt::Format)]
pub enum MotorCommand {
    Move { x: f32, y: f32 },
    Rotate { direction: RobotRotation }, // Speed in percentage
    StopMotors,
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, PartialEq, defmt::Format)]
pub enum EmergeMqttEvent {
    Test
}

impl<const P: usize> FromApplicationMessage<P> for EmergeMqttEvent {
    fn from_application_message(
        message: &ApplicationMessage<P>,
    ) -> Result<Self, EventHandlerError> {
        let received = match message.topic_name {
            "my/topic" => {
                info!("Received message on my/topic: {}", message.payload);
                Ok(EmergeMqttEvent::Test)
            },
            _ => {
                // error!("Unexpected topic: {}", message.topic_name);
                Err(EventHandlerError::UnexpectedApplicationMessageTopic)
            },
        }?;

        Ok(received)
    }
}

#[derive(serde::Serialize, serde::Deserialize, Debug, Clone, PartialEq, defmt::Format)]
pub enum EmergeMqttAction {

}

impl MqttOperations for EmergeMqttAction {
    async fn perform<'a, 'b, C>(
        &'b mut self,
        _client: &mut C,
        _client_id: &'a str,
        _connection_id: ConnectionId,
        _is_retry: bool,
    ) -> Result<(), ClientError>
    where
        C: Client<'a>,
    {
        // match self {
        //     Action::Button(pressed) => {
        //         let payload = if *pressed { "true" } else { "false" };
        //         client
        //             .publish(
        //                 TOPIC_BUTTON,
        //                 payload.as_bytes(),
        //                 QualityOfService::Qos1,
        //                 false,
        //             )
        //             .await?;
        //     }
        // }
        Ok(())
    }
}