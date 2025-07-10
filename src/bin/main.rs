#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(type_alias_impl_trait)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::Config;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{clock::CpuClock, i2c::master::I2c};
use esp_hal::{dma_buffers, Async};
use esp_wifi::init;
use esp_wifi::wifi::event;
use mipidsi::interface::SpiInterface;
use project_emerge_firmware::robot;
use project_emerge_firmware::robot::display_manager::{DisplayManager, ST7789DisplayManager};
use project_emerge_firmware::robot::event_loop::EventLoop;
use project_emerge_firmware::robot::events::{DisplayEvents, RobotEvents};
use static_cell::make_static;

// #[panic_handler]
// fn panic(_: &core::panic::PanicInfo) -> ! {
//     loop {}
// }

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// Commented out for now, will be needed for WiFi functionality
// const SSID: &str = env!("SSID");
// const PASSWORD: &str = env!("PASSWORD");

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 0.4.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    // Create the command channel for inter-task communication
    let command_channel: &'static mut Channel<
        NoopRawMutex,
        robot::events::RobotEvents<'static>,
        20,
    > = make_static!(Channel::new());

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = &*make_static!(init(timer1.timer0, rng.clone(), peripherals.RADIO_CLK)
        .expect("Failed to initialize WIFI/BLE controller"));

    let (_wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");
    // let wifi_interface = interfaces.sta;

    // let config = Config::dhcpv4(Default::default());
    // let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    // let (stack, runner) = embassy_net::new(
    //     wifi_interface,
    //     config,
    //     make_static!(StackResources::<3>::new()),
    //     seed,
    // );

    // spawner.spawn(connection(wifi_controller)).ok();
    // spawner.spawn(net_task(runner)).ok();

    // //wait until wifi connected
    // loop {
    //     info!("Waiting for wifi connection...");
    //     if stack.is_link_up() {
    //         break;
    //     }
    //     Timer::after(Duration::from_millis(500)).await;
    // }

    // info!("Waiting to get IP address...");
    // loop {
    //     if let Some(config) = stack.config_v4() {
    //         info!("Got IP: {}", config.address); //dhcp IP address
    //         break;
    //     }
    //     Timer::after(Duration::from_millis(500)).await;
    // }

    let sda = peripherals.GPIO39;
    let scl = peripherals.GPIO38;

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();

    // Initialize the display
    let sclk = peripherals.GPIO9;
    let mosi = peripherals.GPIO10;
    let cs = Output::new(peripherals.GPIO11, Level::Low, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO46, Level::Low, OutputConfig::default());
    let res = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(60))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi);

    let si = ExclusiveDevice::new(spi, cs, embassy_time::Delay).unwrap();

    let buffer: &'static mut [u8; 512] = make_static!([0_u8; 512]);
    let interface = SpiInterface::new(si, dc, buffer);
    let mut display = ST7789DisplayManager::new(&mut embassy_time::Delay, interface, res).unwrap();

    display.clear_display(Rgb565::BLACK).unwrap();

    let mut event_loop = EventLoop::new(command_channel, display);

    // Event loop creation and spawning
    spawner
        .spawn(monitor_battery_loop(i2c, command_channel))
        .ok();
    spawner.spawn(display_task(command_channel)).ok();

    event_loop.run().await;

    core::panic!("Event loop task returned unexpectedly");

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}

// // maintains wifi connection, when it disconnects it tries to reconnect
// #[embassy_executor::task]
// async fn connection(mut controller: WifiController<'static>) {
//     info!("start connection task");
//     info!("Device capabilities: {:?}", controller.capabilities());
//     loop {
//         match esp_wifi::wifi::wifi_state() {
//             WifiState::StaConnected => {
//                 // wait until we're no longer connected
//                 controller.wait_for_event(WifiEvent::StaDisconnected).await;
//                 Timer::after(Duration::from_millis(5000)).await
//             }
//             _ => {}
//         }
//         if !matches!(controller.is_started(), Ok(true)) {
//             let client_config = Configuration::Client(ClientConfiguration {
//                 ssid: SSID.try_into().unwrap(),
//                 password: PASSWORD.try_into().unwrap(),
//                 ..Default::default()
//             });
//             controller.set_configuration(&client_config).unwrap();
//             info!("Starting wifi");
//             controller.start_async().await.unwrap();
//             info!("Wifi started!");
//         }
//         info!("About to connect...");

//         match controller.connect_async().await {
//             Ok(_) => info!("Wifi connected!"),
//             Err(e) => {
//                 error!("Failed to connect to wifi: {}", e);
//                 Timer::after(Duration::from_millis(5000)).await
//             }
//         }
//     }
// }

// #[embassy_executor::task]
// async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
//     runner.run().await
// }

#[embassy_executor::task]
async fn monitor_battery_loop(
    i2c: I2c<'static, Async>,
    command_sender: &'static Channel<NoopRawMutex, robot::events::RobotEvents<'static>, 20>,
) -> ! {
    info!("Starting battery monitoring loop");
    robot::battery_manager::monitor_battery_loop(i2c, command_sender)
        .await
        .unwrap();
    core::panic!("monitor_battery_loop returned unexpectedly");
}

#[embassy_executor::task]
async fn display_task(
    command_sender: &'static Channel<NoopRawMutex, robot::events::RobotEvents<'static>, 20>,
) {
    loop {
        command_sender
            .send(RobotEvents::Display(DisplayEvents::Render))
            .await;
        Timer::after(Duration::from_secs(2)).await;
    }
}
