//! # TB6612FNG Motor Driver Library
//! 
//! Driver for the TB6612FNG dual H-bridge motor driver.

use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;

/// Maximum PWM duty cycle value (8-bit)
pub const MAX_DUTY_CYCLE: u8 = 255;

/// Minimum PWM duty cycle for reliable motor operation
pub const MIN_DUTY_CYCLE: u8 = 50;

/// Drive commands that can be sent to a motor
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveCommand {
    /// Drive motor forward with specified PWM duty cycle (0-255)
    Forward(u8),
    /// Drive motor backward with specified PWM duty cycle (0-255)
    Backward(u8),
    /// Stop motor (coast to stop)
    Stop,
    /// Brake motor (short brake, immediate stop)
    Brake,
}

impl DriveCommand {
    /// Create a forward command with speed percentage (0.0-1.0)
    pub fn forward_percent(speed: f32) -> Result<Self, MotorError> {
        if !(0.0..=1.0).contains(&speed) {
            return Err(MotorError::InvalidSpeed);
        }
        Ok(DriveCommand::Forward((speed * MAX_DUTY_CYCLE as f32) as u8))
    }

    /// Create a backward command with speed percentage (0.0-1.0)
    pub fn backward_percent(speed: f32) -> Result<Self, MotorError> {
        if !(0.0..=1.0).contains(&speed) {
            return Err(MotorError::InvalidSpeed);
        }
        Ok(DriveCommand::Backward((speed * MAX_DUTY_CYCLE as f32) as u8))
    }

    /// Get the PWM duty cycle for this command
    pub fn duty_cycle(&self) -> u8 {
        match self {
            DriveCommand::Forward(duty) | DriveCommand::Backward(duty) => *duty,
            DriveCommand::Stop | DriveCommand::Brake => 0,
        }
    }

    /// Check if this command will result in motor movement
    pub fn is_moving(&self) -> bool {
        match self {
            DriveCommand::Forward(duty) | DriveCommand::Backward(duty) => *duty > 0,
            DriveCommand::Stop | DriveCommand::Brake => false,
        }
    }
}

/// Errors that can occur during motor operations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorError {
    /// GPIO pin operation failed
    GpioError,
    /// PWM operation failed
    PwmError,
    /// Invalid speed value provided
    InvalidSpeed,
    /// Motor is in standby mode
    MotorInStandby,
    /// Hardware communication error
    HardwareError,
}

impl core::fmt::Display for MotorError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            MotorError::GpioError => write!(f, "GPIO pin operation failed"),
            MotorError::PwmError => write!(f, "PWM operation failed"),
            MotorError::InvalidSpeed => write!(f, "Invalid speed value"),
            MotorError::MotorInStandby => write!(f, "Motor is in standby mode"),
            MotorError::HardwareError => write!(f, "Hardware communication error"),
        }
    }
}

/// Motor state for internal tracking
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorState {
    Stopped,
    Forward,
    Backward,
    Braking,
}

/// Single motor controller for TB6612FNG
#[derive(Debug)]
pub struct Motor<IN1, IN2, PWM, STBY> 
where
    IN1: OutputPin,
    IN2: OutputPin,
    PWM: SetDutyCycle,
    STBY: OutputPin,
{
    in1: IN1,
    in2: IN2,
    pwm: PWM,
    standby: STBY,
    state: MotorState,
    last_duty: u8,
    standby_enabled: bool,
}

impl<IN1, IN2, PWM, STBY> Motor<IN1, IN2, PWM, STBY>
where
    IN1: OutputPin,
    IN2: OutputPin,
    PWM: SetDutyCycle,
    STBY: OutputPin,
{
    /// Create a new motor controller
    pub fn new(
        mut in1: IN1,
        mut in2: IN2,
        mut pwm: PWM,
        mut standby: STBY,
    ) -> Result<Self, MotorError> {
        // Initialize pins to safe state
        in1.set_low().map_err(|_| MotorError::GpioError)?;
        in2.set_low().map_err(|_| MotorError::GpioError)?;
        pwm.set_duty_cycle_percent(0).map_err(|_| MotorError::PwmError)?;
        standby.set_high().map_err(|_| MotorError::GpioError)?; // Active by default

        Ok(Motor {
            in1,
            in2,
            pwm,
            standby,
            state: MotorState::Stopped,
            last_duty: 0,
            standby_enabled: false,
        })
    }

    /// Drive the motor with the specified command
    pub fn drive(&mut self, command: DriveCommand) -> Result<(), MotorError> {
        if self.standby_enabled {
            return Err(MotorError::MotorInStandby);
        }

        match command {
            DriveCommand::Forward(duty) => self.drive_forward(duty),
            DriveCommand::Backward(duty) => self.drive_backward(duty),
            DriveCommand::Stop => self.stop(),
            DriveCommand::Brake => self.brake(),
        }
    }

    /// Enable standby mode (motor will not respond to drive commands)
    pub fn enable_standby(&mut self) -> Result<(), MotorError> {
        self.standby.set_low().map_err(|_| MotorError::GpioError)?;
        self.standby_enabled = true;
        self.state = MotorState::Stopped;
        Ok(())
    }

    /// Disable standby mode (motor will respond to drive commands)
    pub fn disable_standby(&mut self) -> Result<(), MotorError> {
        self.standby.set_high().map_err(|_| MotorError::GpioError)?;
        self.standby_enabled = false;
        Ok(())
    }

    /// Check if motor is in standby mode
    pub fn is_standby(&self) -> bool {
        self.standby_enabled
    }

    /// Get current motor state
    pub fn state(&self) -> MotorState {
        self.state
    }

    /// Get last used duty cycle
    pub fn last_duty_cycle(&self) -> u8 {
        self.last_duty
    }

    /// Gradually change speed to avoid sudden current spikes
    pub fn ramp_to<F>(&mut self, target_command: DriveCommand, step_size: u8, mut delay_fn: F) -> Result<(), MotorError>
    where
        F: FnMut(),
    {
        let target_duty = target_command.duty_cycle();
        let current_duty = self.last_duty;

        if current_duty == target_duty {
            return self.drive(target_command);
        }

        let mut intermediate_duty = current_duty;

        while intermediate_duty != target_duty {
            intermediate_duty = if target_duty > current_duty {
                core::cmp::min(intermediate_duty + step_size, target_duty)
            } else {
                core::cmp::max(intermediate_duty.saturating_sub(step_size), target_duty)
            };

            let intermediate_command = match target_command {
                DriveCommand::Forward(_) => DriveCommand::Forward(intermediate_duty),
                DriveCommand::Backward(_) => DriveCommand::Backward(intermediate_duty),
                _ => target_command,
            };

            self.drive(intermediate_command)?;
            delay_fn();
        }

        Ok(())
    }

    fn drive_forward(&mut self, duty: u8) -> Result<(), MotorError> {
        self.in1.set_high().map_err(|_| MotorError::GpioError)?;
        self.in2.set_low().map_err(|_| MotorError::GpioError)?;
        let duty_percent = (duty as u16 * 100 / 255) as u8;
        self.pwm.set_duty_cycle_percent(duty_percent).map_err(|_| MotorError::PwmError)?;
        
        self.state = MotorState::Forward;
        self.last_duty = duty;
        Ok(())
    }

    fn drive_backward(&mut self, duty: u8) -> Result<(), MotorError> {
        self.in1.set_low().map_err(|_| MotorError::GpioError)?;
        self.in2.set_high().map_err(|_| MotorError::GpioError)?;
        let duty_percent = (duty as u16 * 100 / 255) as u8;
        self.pwm.set_duty_cycle_percent(duty_percent).map_err(|_| MotorError::PwmError)?;
        
        self.state = MotorState::Backward;
        self.last_duty = duty;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), MotorError> {
        self.in1.set_low().map_err(|_| MotorError::GpioError)?;
        self.in2.set_low().map_err(|_| MotorError::GpioError)?;
        self.pwm.set_duty_cycle_percent(0).map_err(|_| MotorError::PwmError)?;
        
        self.state = MotorState::Stopped;
        self.last_duty = 0;
        Ok(())
    }

    fn brake(&mut self) -> Result<(), MotorError> {
        self.in1.set_high().map_err(|_| MotorError::GpioError)?;
        self.in2.set_high().map_err(|_| MotorError::GpioError)?;
        self.pwm.set_duty_cycle_percent(100).map_err(|_| MotorError::PwmError)?;
        
        self.state = MotorState::Braking;
        self.last_duty = 0;
        
        // Brief brake pulse, then stop
        self.stop()
    }
}

/// Dual motor driver for TB6612FNG
#[derive(Debug)]
pub struct DualMotorDriver<AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY>
where
    AIN1: OutputPin,
    AIN2: OutputPin, 
    PWMA: SetDutyCycle,
    BIN1: OutputPin,
    BIN2: OutputPin,
    PWMB: SetDutyCycle,
    STBY: OutputPin,
{
    // Motor A pins
    ain1: AIN1,
    ain2: AIN2,
    pwma: PWMA,
    
    // Motor B pins
    bin1: BIN1,
    bin2: BIN2,
    pwmb: PWMB,
    
    // Shared standby
    standby: STBY,
    
    // State tracking
    motor_a_state: MotorState,
    motor_b_state: MotorState,
    motor_a_duty: u8,
    motor_b_duty: u8,
    standby_enabled: bool,
}

impl<AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY> DualMotorDriver<AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY>
where
    AIN1: OutputPin,
    AIN2: OutputPin,
    PWMA: SetDutyCycle,
    BIN1: OutputPin,
    BIN2: OutputPin,
    PWMB: SetDutyCycle,
    STBY: OutputPin,
{
    /// Create a new dual motor driver
    pub fn new(
        mut ain1: AIN1,
        mut ain2: AIN2,
        mut pwma: PWMA,
        mut bin1: BIN1,
        mut bin2: BIN2,
        mut pwmb: PWMB,
        mut standby: STBY,
    ) -> Result<Self, MotorError> {
        // Initialize all pins to safe state
        ain1.set_low().map_err(|_| MotorError::GpioError)?;
        ain2.set_low().map_err(|_| MotorError::GpioError)?;
        pwma.set_duty_cycle_percent(0).map_err(|_| MotorError::PwmError)?;
        
        bin1.set_low().map_err(|_| MotorError::GpioError)?;
        bin2.set_low().map_err(|_| MotorError::GpioError)?;
        pwmb.set_duty_cycle_percent(0).map_err(|_| MotorError::PwmError)?;
        
        standby.set_high().map_err(|_| MotorError::GpioError)?;

        Ok(DualMotorDriver {
            ain1,
            ain2,
            pwma,
            bin1,
            bin2,
            pwmb,
            standby,
            motor_a_state: MotorState::Stopped,
            motor_b_state: MotorState::Stopped,
            motor_a_duty: 0,
            motor_b_duty: 0,
            standby_enabled: false,
        })
    }

    /// Drive motor A with the specified command
    pub fn drive_motor_a(&mut self, command: DriveCommand) -> Result<(), MotorError> {
        if self.standby_enabled {
            return Err(MotorError::MotorInStandby);
        }

        match command {
            DriveCommand::Forward(duty) => self.drive_a_forward(duty),
            DriveCommand::Backward(duty) => self.drive_a_backward(duty),
            DriveCommand::Stop => self.stop_a(),
            DriveCommand::Brake => self.brake_a(),
        }
    }

    /// Drive motor B with the specified command
    pub fn drive_motor_b(&mut self, command: DriveCommand) -> Result<(), MotorError> {
        if self.standby_enabled {
            return Err(MotorError::MotorInStandby);
        }

        match command {
            DriveCommand::Forward(duty) => self.drive_b_forward(duty),
            DriveCommand::Backward(duty) => self.drive_b_backward(duty),
            DriveCommand::Stop => self.stop_b(),
            DriveCommand::Brake => self.brake_b(),
        }
    }

    /// Drive both motors simultaneously
    pub fn drive_both(&mut self, cmd_a: DriveCommand, cmd_b: DriveCommand) -> Result<(), MotorError> {
        self.drive_motor_a(cmd_a)?;
        self.drive_motor_b(cmd_b)?;
        Ok(())
    }

    /// Stop both motors
    pub fn stop_all(&mut self) -> Result<(), MotorError> {
        self.stop_a()?;
        self.stop_b()?;
        Ok(())
    }

    /// Brake both motors
    pub fn brake_all(&mut self) -> Result<(), MotorError> {
        self.brake_a()?;
        self.brake_b()?;
        Ok(())
    }

    /// Enable standby mode for both motors
    pub fn enable_standby(&mut self) -> Result<(), MotorError> {
        self.standby.set_low().map_err(|_| MotorError::GpioError)?;
        self.standby_enabled = true;
        self.motor_a_state = MotorState::Stopped;
        self.motor_b_state = MotorState::Stopped;
        Ok(())
    }

    /// Disable standby mode for both motors
    pub fn disable_standby(&mut self) -> Result<(), MotorError> {
        self.standby.set_high().map_err(|_| MotorError::GpioError)?;
        self.standby_enabled = false;
        Ok(())
    }

    /// Check if motors are in standby mode
    pub fn is_standby(&self) -> bool {
        self.standby_enabled
    }

    /// Get motor A state
    pub fn motor_a_state(&self) -> MotorState {
        self.motor_a_state
    }

    /// Get motor B state  
    pub fn motor_b_state(&self) -> MotorState {
        self.motor_b_state
    }

    // Private motor A control methods
    fn drive_a_forward(&mut self, duty: u8) -> Result<(), MotorError> {
        self.ain1.set_high().map_err(|_| MotorError::GpioError)?;
        self.ain2.set_low().map_err(|_| MotorError::GpioError)?;
        let duty_percent = (duty as u16 * 100 / 255) as u8;
        self.pwma.set_duty_cycle_percent(duty_percent).map_err(|_| MotorError::PwmError)?;
        
        self.motor_a_state = MotorState::Forward;
        self.motor_a_duty = duty;
        Ok(())
    }

    fn drive_a_backward(&mut self, duty: u8) -> Result<(), MotorError> {
        self.ain1.set_low().map_err(|_| MotorError::GpioError)?;
        self.ain2.set_high().map_err(|_| MotorError::GpioError)?;
        let duty_percent = (duty as u16 * 100 / 255) as u8;
        self.pwma.set_duty_cycle_percent(duty_percent).map_err(|_| MotorError::PwmError)?;
        
        self.motor_a_state = MotorState::Backward;
        self.motor_a_duty = duty;
        Ok(())
    }

    fn stop_a(&mut self) -> Result<(), MotorError> {
        self.ain1.set_low().map_err(|_| MotorError::GpioError)?;
        self.ain2.set_low().map_err(|_| MotorError::GpioError)?;
        self.pwma.set_duty_cycle_percent(0).map_err(|_| MotorError::PwmError)?;
        
        self.motor_a_state = MotorState::Stopped;
        self.motor_a_duty = 0;
        Ok(())
    }

    fn brake_a(&mut self) -> Result<(), MotorError> {
        self.ain1.set_high().map_err(|_| MotorError::GpioError)?;
        self.ain2.set_high().map_err(|_| MotorError::GpioError)?;
        self.pwma.set_duty_cycle_percent(100).map_err(|_| MotorError::PwmError)?;
        
        self.motor_a_state = MotorState::Braking;
        self.stop_a()
    }

    // Private motor B control methods  
    fn drive_b_forward(&mut self, duty: u8) -> Result<(), MotorError> {
        self.bin1.set_high().map_err(|_| MotorError::GpioError)?;
        self.bin2.set_low().map_err(|_| MotorError::GpioError)?;
        let duty_percent = (duty as u16 * 100 / 255) as u8;
        self.pwmb.set_duty_cycle_percent(duty_percent).map_err(|_| MotorError::PwmError)?;
        
        self.motor_b_state = MotorState::Forward;
        self.motor_b_duty = duty;
        Ok(())
    }

    fn drive_b_backward(&mut self, duty: u8) -> Result<(), MotorError> {
        self.bin1.set_low().map_err(|_| MotorError::GpioError)?;
        self.bin2.set_high().map_err(|_| MotorError::GpioError)?;
        let duty_percent = (duty as u16 * 100 / 255) as u8;
        self.pwmb.set_duty_cycle_percent(duty_percent).map_err(|_| MotorError::PwmError)?;
        
        self.motor_b_state = MotorState::Backward;
        self.motor_b_duty = duty;
        Ok(())
    }

    fn stop_b(&mut self) -> Result<(), MotorError> {
        self.bin1.set_low().map_err(|_| MotorError::GpioError)?;
        self.bin2.set_low().map_err(|_| MotorError::GpioError)?;
        self.pwmb.set_duty_cycle_percent(0).map_err(|_| MotorError::PwmError)?;
        
        self.motor_b_state = MotorState::Stopped;
        self.motor_b_duty = 0;
        Ok(())
    }

    fn brake_b(&mut self) -> Result<(), MotorError> {
        self.bin1.set_high().map_err(|_| MotorError::GpioError)?;
        self.bin2.set_high().map_err(|_| MotorError::GpioError)?;
        self.pwmb.set_duty_cycle_percent(100).map_err(|_| MotorError::PwmError)?;
        
        self.motor_b_state = MotorState::Braking;
        self.stop_b()
    }
}

/// Convenience functions for common motor control patterns
pub mod presets {
    use super::{DriveCommand, DualMotorDriver, MotorError};
    use embedded_hal::digital::OutputPin;
    use embedded_hal::pwm::SetDutyCycle;

    /// Robot movement presets for differential drive robots
    impl<AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY> DualMotorDriver<AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY>
    where
        AIN1: OutputPin,
        AIN2: OutputPin,
        PWMA: SetDutyCycle,
        BIN1: OutputPin,
        BIN2: OutputPin,
        PWMB: SetDutyCycle,
        STBY: OutputPin,
    {
        /// Move robot forward
        pub fn move_forward(&mut self, speed: u8) -> Result<(), MotorError> {
            self.drive_both(DriveCommand::Forward(speed), DriveCommand::Forward(speed))
        }

        /// Move robot backward
        pub fn move_backward(&mut self, speed: u8) -> Result<(), MotorError> {
            self.drive_both(DriveCommand::Backward(speed), DriveCommand::Backward(speed))
        }

        /// Turn robot left (motor A backward, motor B forward)
        pub fn turn_left(&mut self, speed: u8) -> Result<(), MotorError> {
            self.drive_both(DriveCommand::Backward(speed), DriveCommand::Forward(speed))
        }

        /// Turn robot right (motor A forward, motor B backward)
        pub fn turn_right(&mut self, speed: u8) -> Result<(), MotorError> {
            self.drive_both(DriveCommand::Forward(speed), DriveCommand::Backward(speed))
        }

        /// Pivot left (only motor B moves forward)
        pub fn pivot_left(&mut self, speed: u8) -> Result<(), MotorError> {
            self.drive_both(DriveCommand::Stop, DriveCommand::Forward(speed))
        }

        /// Pivot right (only motor A moves forward)  
        pub fn pivot_right(&mut self, speed: u8) -> Result<(), MotorError> {
            self.drive_both(DriveCommand::Forward(speed), DriveCommand::Stop)
        }

        /// Gentle curve left (reduce left motor speed)
        pub fn curve_left(&mut self, base_speed: u8, reduction: u8) -> Result<(), MotorError> {
            let left_speed = base_speed.saturating_sub(reduction);
            self.drive_both(DriveCommand::Forward(left_speed), DriveCommand::Forward(base_speed))
        }

        /// Gentle curve right (reduce right motor speed)
        pub fn curve_right(&mut self, base_speed: u8, reduction: u8) -> Result<(), MotorError> {
            let right_speed = base_speed.saturating_sub(reduction);
            self.drive_both(DriveCommand::Forward(base_speed), DriveCommand::Forward(right_speed))
        }
    }
}

#[cfg(test)]
mod tests {

    #[test]
    fn test_drive_command_creation() {
        let forward = DriveCommand::forward_percent(0.5).unwrap();
        assert_eq!(forward, DriveCommand::Forward(127));

        let backward = DriveCommand::backward_percent(1.0).unwrap();
        assert_eq!(backward, DriveCommand::Backward(255));

        assert!(DriveCommand::forward_percent(1.5).is_err());
        assert!(DriveCommand::backward_percent(-0.1).is_err());
    }

    #[test]
    fn test_drive_command_properties() {
        let forward = DriveCommand::Forward(100);
        assert_eq!(forward.duty_cycle(), 100);
        assert!(forward.is_moving());

        let stop = DriveCommand::Stop;
        assert_eq!(stop.duty_cycle(), 0);
        assert!(!stop.is_moving());
    }

    #[test]
    fn test_motor_error_display() {
        let error = MotorError::InvalidSpeed;
        assert_eq!(error.to_string(), "Invalid speed value");
    }
}