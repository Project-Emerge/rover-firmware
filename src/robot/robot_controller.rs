use embedded_hal::pwm::SetDutyCycle;
use esp_idf_svc::hal::gpio::{Output, Pin, PinDriver};
use tb6612fng::Tb6612fng;

type Motors<
    AIn1: Pin,
    AIn2: Pin,
    APwm: SetDutyCycle,
    BIn1: Pin,
    BIn2: Pin,
    BPwm: SetDutyCycle,
    Standby: Pin,
> = Tb6612fng<
    PinDriver<'static, AIn1, Output>,
    PinDriver<'static, AIn2, Output>,
    APwm,
    PinDriver<'static, BIn1, Output>,
    PinDriver<'static, BIn2, Output>,
    BPwm,
    PinDriver<'static, Standby, Output>,
>;

/// Robot controller implementation using TB6612FNG motor drivers.
///
/// This implementation provides motor control for a differential drive robot
/// using two TB6612FNG dual motor driver chips - one for left side motors
/// and one for right side motors.
pub struct RobotControllerTB6612FNG<
    AIn1Left: Pin,
    AIn2Left: Pin,
    APwmLeft: SetDutyCycle,
    BIn1Left: Pin,
    BIn2Left: Pin,
    BPwmLeft: SetDutyCycle,
    StandbyLeft: Pin,
    AIn1Right: Pin,
    AIn2Right: Pin,
    APwmRight: SetDutyCycle,
    BIn1Right: Pin,
    BIn2Right: Pin,
    BPwmRight: SetDutyCycle,
    StandbyRight: Pin,
> {
    motors_left: Motors<AIn1Left, AIn2Left, APwmLeft, BIn1Left, BIn2Left, BPwmLeft, StandbyLeft>,
    motors_right:
        Motors<AIn1Right, AIn2Right, APwmRight, BIn1Right, BIn2Right, BPwmRight, StandbyRight>,
}
