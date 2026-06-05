//! Demo test suite using embedded-test
//!
//! You can run this using `cargo test` as usual.

#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;
    use esp_hal::interrupt::software::SoftwareInterruptControl;
    use esp_hal::timer::systimer::SystemTimer;

    #[init]
    fn init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let timer0 = SystemTimer::new(peripherals.SYSTIMER);
        let software_interrupts = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
        esp_rtos::start(timer0.alarm0, software_interrupts.software_interrupt0);

        rtt_target::rtt_init_defmt!();
    }

    #[test]
    async fn hello_test() {
        defmt::info!("Running test!");

        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        assert_eq!(1 + 1, 2);
    }
}
