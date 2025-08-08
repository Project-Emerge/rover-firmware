#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(type_alias_impl_trait)]

use alloc::string::ToString;
use core::fmt::Write;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_net::IpAddress;
use embassy_net::{Config, Runner, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Timer};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::RgbColor;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::mcpwm::operator::PwmPinConfig;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::spi::master::{Config as SpiConfig, Spi};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;
use esp_hal::{clock::CpuClock, i2c::master::I2c};
use esp_wifi::wifi::{
    ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiState,
};
use esp_wifi::{init, EspWifiController};
use heapless::String;
use mipidsi::interface::SpiInterface;
use project_emerge_firmware::robot;
use project_emerge_firmware::robot::display_manager::{DisplayManager, ST7789DisplayManager};
use project_emerge_firmware::robot::event_loop::EventLoop;
use project_emerge_firmware::robot::events::{DisplayEvents, RobotEvents};
use project_emerge_firmware::robot::motors_manager::Tb6612fngMotorsManager;
use project_emerge_firmware::utils::mqtt_manager::{self, EmergeMqttAction};
use smoltcp::wire::DnsQueryType;
use static_cell::StaticCell;

use project_emerge_firmware::utils::channels::{ActionChannel, ActionPub, EventChannel, EventPub};
// use static_cell::make_static;

// #[panic_handler]
// fn panic(_: &core::panic::PanicInfo) -> ! {
//     loop {}
// }

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const MQTT_BROKER: &str = env!("MQTT_BROKER");
const MQTT_PORT: &str = env!("MQTT_PORT");

static UID: StaticCell<String<64>> = StaticCell::new();

static EVENT_CHANNEL: StaticCell<EventChannel> = StaticCell::new();
static ACTION_CHANNEL: StaticCell<ActionChannel> = StaticCell::new();

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

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);

    // Create the command channel for inter-task communication
    // let command_channel: &'static mut Channel<NoopRawMutex, robot::events::RobotEvents, 64> = mk_static!(
    //     Channel<NoopRawMutex, robot::events::RobotEvents, 64>,
    //     Channel::new()
    // );

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init = &*mk_static!(
        EspWifiController<'static>,
        init(timer1.timer0, rng.clone()).expect("Failed to initialize WIFI/BLE controller")
    );

    let (wifi_controller, interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");
    let wifi_interface = interfaces.sta;

    let config = Config::dhcpv4(Default::default());
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources::<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(wifi_controller)).ok();
    spawner.spawn(net_task(runner)).ok();

    let event_channel =
        EVENT_CHANNEL.init(PubSubChannel::<NoopRawMutex, RobotEvents, 16, 4, 4>::new());
    let event_sub = event_channel.subscriber().unwrap();
    // let event_pub = event_channel.publisher().unwrap();

    let action_channel =
        ACTION_CHANNEL.init(PubSubChannel::<NoopRawMutex, EmergeMqttAction, 16, 4, 4>::new());
    let action_pub = action_channel.publisher().unwrap();
    let action_sub = action_channel.subscriber().unwrap();

    // Create a publisher for wait_for_connection
    let event_pub_for_wait = event_channel.publisher().unwrap();
    wait_for_connection(stack, event_pub_for_wait).await;

    let uid_handle = UID.init(String::new());
    core::write!(uid_handle, "project-emerge-{}", "12345").unwrap();

    let IpAddress::Ipv4(address) = match MQTT_BROKER.parse::<IpAddress>() {
        Ok(addr) => addr,
        Err(_) => stack
            .dns_query(MQTT_BROKER, DnsQueryType::A)
            .await
            .map(|a| a[0])
            .expect("Failed to resolve MQTT broker address"),
    };

    // Create a publisher for mqtt_manager::init
    let event_pub_for_mqtt = event_channel.publisher().unwrap();
    mqtt_manager::init(
        &spawner,
        stack,
        uid_handle,
        event_pub_for_mqtt,
        action_sub,
        address,
        MQTT_PORT.parse::<u16>().unwrap(),
    )
    .await;

    info!("MQTT manager initialized");

    let sda = peripherals.GPIO39;
    let scl = peripherals.GPIO38;

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
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

    let mut buffer = [0_u8; 512];
    let interface = SpiInterface::new(si, dc, &mut buffer);
    let mut display = ST7789DisplayManager::new(&mut embassy_time::Delay, interface, res).unwrap();

    display.clear_display(Rgb565::BLACK).unwrap();

    let ain1_1 = Output::new(peripherals.GPIO15, Level::Low, OutputConfig::default());
    let ain2_1 = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());
    let bin1_1 = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    let bin2_1 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let stbya = Output::new(peripherals.GPIO12, Level::Low, OutputConfig::default());
    let operator0 = mcpwm.operator0;
    let (pwma_1, pwmb_1) = operator0.with_pins(
        peripherals.GPIO17,
        PwmPinConfig::UP_ACTIVE_HIGH,
        peripherals.GPIO5,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    let ain1_2 = Output::new(peripherals.GPIO21, Level::Low, OutputConfig::default());
    let ain2_2 = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());
    let bin1_2 = Output::new(peripherals.GPIO47, Level::Low, OutputConfig::default());
    let bin2_2 = Output::new(peripherals.GPIO48, Level::Low, OutputConfig::default());
    let stbyb = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let operator1 = mcpwm.operator1;
    let (pwma_2, pwmb_2) = operator1.with_pins(
        peripherals.GPIO13,
        PwmPinConfig::UP_ACTIVE_HIGH,
        peripherals.GPIO45,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    let timer_clock_cfg =
        clock_cfg.timer_clock_with_frequency(99, PwmWorkingMode::Increase, Rate::from_khz(20)).unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let motor_driver = Tb6612fngMotorsManager::new(
        ain1_1, ain2_1, pwma_1, bin1_1, bin2_1, pwmb_1, stbya, ain1_2, ain2_2, pwma_2, bin1_2,
        bin2_2, pwmb_2, stbyb,
    )
    .unwrap();

    let signal = mk_static!(
        embassy_sync::signal::Signal<NoopRawMutex, ()>,
        embassy_sync::signal::Signal::new()
    );
    let mut event_loop = EventLoop::new(
        display,
        motor_driver,
        signal,
        event_sub,
        action_pub,
        spawner,
    );

    // Event loop creation and spawning
    spawner
        .spawn(monitor_battery_loop(
            i2c,
            event_channel.publisher().unwrap(),
            action_channel.publisher().unwrap(),
        ))
        .ok();
    spawner
        .spawn(display_task(event_channel.publisher().unwrap()))
        .ok();
    spawner
        .spawn(system_health_monitor(event_channel.publisher().unwrap()))
        .ok();

    event_loop.run().await;

    core::panic!("Event loop task returned unexpectedly");

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.1/examples/src/bin
}

async fn wait_for_connection(stack: Stack<'_>, command_sender: EventPub) {
    info!("Waiting for link to be up");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            let ip = heapless::String::try_from(config.address.to_string().as_str()).unwrap();
            command_sender
                .publish(RobotEvents::Display(DisplayEvents::ShowIp { ip }))
                .await;
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

// maintains wifi connection, when it disconnects it tries to reconnect
#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("start connection task");
    // info!("Device capabilities: {:?}", controller.capabilities().unwrap());
    loop {
        match esp_wifi::wifi::wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            info!("Starting wifi");
            controller.start_async().await.unwrap();
            info!("Wifi started!");
        }
        info!("About to connect...");

        match embassy_time::with_timeout(Duration::from_secs(30), controller.connect_async()).await
        {
            Ok(Ok(_)) => info!("Wifi connected!"),
            Ok(Err(e)) => {
                error!("Failed to connect to wifi: {}", e);
                Timer::after(Duration::from_millis(5000)).await
            }
            Err(_) => {
                error!("Wifi connection timeout");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn monitor_battery_loop(
    i2c: I2c<'static, Async>,
    event_pub: EventPub,
    action_pub: ActionPub,
) -> ! {
    info!("Starting battery monitoring loop");
    robot::battery_manager::monitor_battery_loop(i2c, event_pub, action_pub)
        .await
        .unwrap();
    core::panic!("monitor_battery_loop returned unexpectedly");
}

#[embassy_executor::task]
async fn display_task(command_sender: EventPub) {
    loop {
        command_sender
            .publish(RobotEvents::Display(DisplayEvents::Render))
            .await;
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[embassy_executor::task]
async fn system_health_monitor(command_sender: EventPub) {
    let mut last_heap_info_time = embassy_time::Instant::now();

    loop {
        // Log system health information every 30 seconds
        if embassy_time::Instant::now().duration_since(last_heap_info_time)
            >= Duration::from_secs(30)
        {
            let free_heap = esp_alloc::HEAP.free();
            let used_heap = esp_alloc::HEAP.used();
            let total_heap = free_heap + used_heap;

            info!(
                "Heap status - Free: {} bytes, Used: {} bytes, Total: {} bytes",
                free_heap, used_heap, total_heap
            );

            command_sender
                .publish(RobotEvents::Display(DisplayEvents::ShowMemoryStats {
                    free_memory: free_heap as f32 / 1024.0,
                    total_memory: total_heap as f32 / 1024.0,
                }))
                .await;

            // Check if we're getting low on memory
            if free_heap < 8192 {
                // Less than 8KB free
                error!("Low memory warning: only {} bytes free", free_heap);
            }

            last_heap_info_time = embassy_time::Instant::now();
        }

        Timer::after(Duration::from_secs(10)).await;
    }
}
