use defmt::info;
use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};
use tb6612fng::{DriveCommand, Motor, Tb6612fng, Tb6612fngError};

#[derive(Debug, Clone, defmt::Format)]
pub enum MotorError {
    StandbyError,
}

impl<Stby> From<Tb6612fngError<Stby>> for MotorError {
    fn from(error: Tb6612fngError<Stby>) -> Self {
        match error {
            Tb6612fngError::Standby(_) => MotorError::StandbyError,
        }
    }
}

pub trait MotorsManager {
    fn drive(&mut self, left: f32, right: f32) -> Result<(), MotorError>;
    fn forward(&mut self, speed: f32) -> Result<(), MotorError>;
    fn backward(&mut self, speed: f32) -> Result<(), MotorError>;
    fn stop(&mut self) -> Result<(), MotorError>;
    fn rotate_clockwise(&mut self, speed: f32) -> Result<(), MotorError>;
    fn rotate_counter_clockwise(&mut self, speed: f32) -> Result<(), MotorError>;
}

pub struct Tb6612fngMotorsManager<
    RLIn1,
    RLIn2,
    RLPwm,
    RRIn1,
    RRIn2,
    RRPwm,
    RStandby,
    FLIn1,
    FLIn2,
    FLPwm,
    FRIn1,
    FRIn2,
    FRPwm,
    FStandby,
> where
    RLIn1: OutputPin,
    RLIn2: OutputPin,
    RLPwm: SetDutyCycle,
    RRIn1: OutputPin,
    RRIn2: OutputPin,
    RRPwm: SetDutyCycle,
    RStandby: OutputPin,
    FLIn1: OutputPin,
    FLIn2: OutputPin,
    FLPwm: SetDutyCycle,
    FRIn1: OutputPin,
    FRIn2: OutputPin,
    FRPwm: SetDutyCycle,
    FStandby: OutputPin,
{
    rear_motors: Tb6612fng<RLIn1, RLIn2, RLPwm, RRIn1, RRIn2, RRPwm, RStandby>,
    front_motors: Tb6612fng<FLIn1, FLIn2, FLPwm, FRIn1, FRIn2, FRPwm, FStandby>,
}

impl<
        RLIn1,
        RLIn2,
        RLPwm,
        RRIn1,
        RRIn2,
        RRPwm,
        RStandby,
        FLIn1,
        FLIn2,
        FLPwm,
        FRIn1,
        FRIn2,
        FRPwm,
        FStandby,
    >
    Tb6612fngMotorsManager<
        RLIn1,
        RLIn2,
        RLPwm,
        RRIn1,
        RRIn2,
        RRPwm,
        RStandby,
        FLIn1,
        FLIn2,
        FLPwm,
        FRIn1,
        FRIn2,
        FRPwm,
        FStandby,
    >
where
    RLIn1: OutputPin,
    RLIn2: OutputPin,
    RLPwm: SetDutyCycle,
    RRIn1: OutputPin,
    RRIn2: OutputPin,
    RRPwm: SetDutyCycle,
    RStandby: OutputPin,
    FLIn1: OutputPin,
    FLIn2: OutputPin,
    FLPwm: SetDutyCycle,
    FRIn1: OutputPin,
    FRIn2: OutputPin,
    FRPwm: SetDutyCycle,
    FStandby: OutputPin,
{
    pub fn new(
        rlin1: RLIn1,
        rlin2: RLIn2,
        rlpwm: RLPwm,
        rrin1: RRIn1,
        rrin2: RRIn2,
        rrpwm: RRPwm,
        rstandby: RStandby,
        flin1: FLIn1,
        flin2: FLIn2,
        flpwm: FLPwm,
        frin1: FRIn1,
        frin2: FRIn2,
        frpwm: FRPwm,
        fstandby: FStandby,
    ) -> Result<Self, MotorError> {
        let rear_left_motor =
            Motor::new(rlin1, rlin2, rlpwm).map_err(|_| MotorError::StandbyError)?;
        let rear_right_motor =
            Motor::new(rrin1, rrin2, rrpwm).map_err(|_| MotorError::StandbyError)?;
        let rear_motors = Tb6612fng::new(rear_left_motor, rear_right_motor, rstandby)?;

        let front_left_motor =
            Motor::new(flin1, flin2, flpwm).map_err(|_| MotorError::StandbyError)?;
        let front_right_motor =
            Motor::new(frin1, frin2, frpwm).map_err(|_| MotorError::StandbyError)?;
        let front_motors = Tb6612fng::new(front_left_motor, front_right_motor, fstandby)?;

        Ok(Tb6612fngMotorsManager {
            rear_motors,
            front_motors,
        })
    }

    #[allow(dead_code)]
    fn drive_left_motors(&mut self, command: DriveCommand) -> Result<(), MotorError> {
        self.rear_motors.motor_a.drive(command).map_err(|_| MotorError::StandbyError)?;
        self.front_motors.motor_a.drive(command).map_err(|_| MotorError::StandbyError)?;
        Ok(())
    }

    #[allow(dead_code)]
    fn drive_right_motors(&mut self, command: DriveCommand) -> Result<(), MotorError> {
        self.rear_motors.motor_b.drive(command).map_err(|_| MotorError::StandbyError)?;
        self.front_motors.motor_b.drive(command).map_err(|_| MotorError::StandbyError)?;
        Ok(())
    }
}

impl<
        RLIn1,
        RLIn2,
        RLPwm,
        RRIn1,
        RRIn2,
        RRPwm,
        RStandby,
        FLIn1,
        FLIn2,
        FLPwm,
        FRIn1,
        FRIn2,
        FRPwm,
        FStandby,
    > MotorsManager for
    Tb6612fngMotorsManager<
        RLIn1,
        RLIn2,
        RLPwm,
        RRIn1,
        RRIn2,
        RRPwm,
        RStandby,
        FLIn1,
        FLIn2,
        FLPwm,
        FRIn1,
        FRIn2,
        FRPwm,
        FStandby,
    >
where
    RLIn1: OutputPin,
    RLIn2: OutputPin,
    RLPwm: SetDutyCycle,
    RRIn1: OutputPin,
    RRIn2: OutputPin,
    RRPwm: SetDutyCycle,
    RStandby: OutputPin,
    FLIn1: OutputPin,
    FLIn2: OutputPin,
    FLPwm: SetDutyCycle,
    FRIn1: OutputPin,
    FRIn2: OutputPin,
    FRPwm: SetDutyCycle,
    FStandby: OutputPin,
{
    fn drive(&mut self, left: f32, right: f32) -> Result<(), MotorError> {
        // left and right are expected to be in the range [-1.0, 1.0]
        let left_speed: u8 = (left.abs() * 100.0) as u8;
        let right_speed: u8 = (right.abs() * 100.0) as u8;
        info!("Driving with left: {}, right: {}", left_speed, right_speed);
        
        let left_command = if left >= 0.0 { DriveCommand::Forward(left_speed) } else { DriveCommand::Backward(left_speed) };
        let right_command = if right >= 0.0 { DriveCommand::Forward(right_speed) } else { DriveCommand::Backward(right_speed) };

        self.drive_left_motors(left_command)?;
        self.drive_right_motors(right_command)?;

        Ok(())
    }

    fn forward(&mut self, speed: f32) -> Result<(), MotorError> {
        self.drive(speed, speed)
    }

    fn backward(&mut self, speed: f32) -> Result<(), MotorError> {
        self.drive(-speed, -speed)
    }

    fn stop(&mut self) -> Result<(), MotorError> {
        self.rear_motors.motor_a.drive(DriveCommand::Stop).map_err(|_| MotorError::StandbyError)?;
        self.front_motors.motor_a.drive(DriveCommand::Stop).map_err(|_| MotorError::StandbyError)?;
        self.rear_motors.motor_b.drive(DriveCommand::Stop).map_err(|_| MotorError::StandbyError)?;
        self.front_motors.motor_b.drive(DriveCommand::Stop).map_err(|_| MotorError::StandbyError)?;
        Ok(())
    }

    fn rotate_clockwise(&mut self, speed: f32) -> Result<(), MotorError> {
        self.drive(speed, -speed)
    }

    fn rotate_counter_clockwise(&mut self, speed: f32) -> Result<(), MotorError> {
        self.drive(-speed, speed)
    }
}
