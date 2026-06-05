#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_bootloader_esp_idf as _;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;

esp_bootloader_esp_idf::esp_app_desc!();

#[embassy_executor::task]
async fn hello_task() -> ! {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        info!("Hello, world!");
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(esp_hal::Config::default());
    info!("RTT logging initialized");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    info!("Spawning hello task");
    spawner.spawn(hello_task().unwrap());
    info!("Hello task spawned");

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
