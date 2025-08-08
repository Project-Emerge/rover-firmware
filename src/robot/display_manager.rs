use core::fmt::Write;
use embassy_time::Delay;
use embedded_graphics::{
    mono_font::{iso_8859_10::FONT_9X15, iso_8859_2::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::{Point, RgbColor, *},
    primitives::{Circle, PrimitiveStyle},
    text::{Alignment, Text},
};
use embedded_hal::digital::OutputPin;
use heapless::String;
use mipidsi::{
    interface::{Interface, InterfacePixelFormat},
    models::ST7789,
    options::{ColorInversion, Orientation, Rotation},
    Builder, Display,
};

#[derive(Debug, Clone)]
pub enum DisplayError {
    InitializationError,
    DrawError,
    WriteError,
    TextRenderError,
    UpdateError(heapless::String<128>),
}

type DisplayResult<T> = Result<T, DisplayError>;

pub trait DisplayManager {
    fn clear_display(&mut self, color: Rgb565) -> DisplayResult<()>;
    fn write_current_value(&mut self, value: i64) -> DisplayResult<()>;
    fn write_battery_voltage(&mut self, voltage: f32) -> DisplayResult<()>;
    fn write_battery_percentage(&mut self, percentage: u8) -> DisplayResult<()>;
    fn show_wifi_status(&mut self, ip: heapless::String<64>) -> DisplayResult<()>;
    fn write_mqtt_status(&mut self, connected: bool) -> DisplayResult<()>;
    fn write_memory_stats(&mut self, free_memory: f32, total_memory: f32) -> DisplayResult<()>;
    fn show(&mut self) -> DisplayResult<()>;
}

pub struct ST7789DisplayManager<'a, DI, Rst>
where
    DI: Interface,
    Rst: OutputPin,
    Rgb565: InterfacePixelFormat<DI::Word>,
{
    display: Display<DI, ST7789, Rst>,
    text_style: MonoTextStyle<'a, Rgb565>,
    // display values
    current_value: i64,
    battery_voltage: f32,
    battery_percentage: u8,
    wifi_ip: String<64>,
    mqtt_connected: bool,
    free_memory: f32,
    total_memory: f32,
    device_id: i32,
}

impl<'a, DI, Rst> ST7789DisplayManager<'a, DI, Rst>
where
    DI: Interface,
    Rgb565: InterfacePixelFormat<DI::Word>,
    Rst: OutputPin,
{
    pub fn new(delay: &mut Delay, spi: DI, rst_pin: Rst) -> DisplayResult<Self> {
        let display = Builder::new(ST7789, spi)
            .reset_pin(rst_pin)
            .display_size(135, 240)
            .display_offset(52, 40)
            .invert_colors(ColorInversion::Inverted)
            .orientation(Orientation::new().rotate(Rotation::Deg90))
            .init(delay)
            .map_err(|_| DisplayError::InitializationError)?;
        Ok(ST7789DisplayManager {
            display,
            text_style: MonoTextStyle::new(&FONT_9X15, Rgb565::WHITE),
            current_value: 0,
            battery_voltage: 0.0,
            battery_percentage: u8::MIN,
            wifi_ip: String::try_from("Not connected").unwrap(),
            mqtt_connected: false,
            free_memory: 0.0,
            total_memory: 0.0,
            device_id: env!("ROBOT_ID").parse::<i32>().unwrap(), // Make sure ROBOT_ID is set to a valid integer string
        })
    }

    /// Get display dimensions
    pub fn get_dimensions(&self) -> (u32, u32) {
        // ST7789 with 90-degree rotation: width=240, height=135
        (240, 135)
    }
}

impl<'a, DI, Rst> DisplayManager for ST7789DisplayManager<'a, DI, Rst>
where
    DI: Interface,
    Rgb565: InterfacePixelFormat<DI::Word>,
    Rst: OutputPin,
{
    fn clear_display(&mut self, color: Rgb565) -> DisplayResult<()> {
        self.display
            .clear(color)
            .map_err(|_| DisplayError::DrawError)
    }
    /// value is in mA
    fn write_current_value(&mut self, value: i64) -> DisplayResult<()> {
        if value >= 0 {
            self.current_value = value;
            Ok(())
        } else {
            return Err(DisplayError::UpdateError(
                heapless::String::try_from("Invalid current value").unwrap(),
            ));
        }
    }

    fn write_battery_voltage(&mut self, voltage: f32) -> DisplayResult<()> {
        if voltage >= 0.0 {
            self.battery_voltage = voltage;
            Ok(())
        } else {
            Err(DisplayError::UpdateError(
                heapless::String::try_from("Invalid voltage value").unwrap(),
            ))
        }
    }

    fn write_battery_percentage(&mut self, percentage: u8) -> DisplayResult<()> {
        if percentage <= 100 {
            self.battery_percentage = percentage;
            Ok(())
        } else {
            Err(DisplayError::UpdateError(
                heapless::String::try_from("Invalid battery percentage").unwrap(),
            ))
        }
    }

    fn show_wifi_status(&mut self, ip: heapless::String<64>) -> DisplayResult<()> {
        if ip.is_empty() {
            Err(DisplayError::UpdateError(
                heapless::String::try_from("Invalid WiFi status").unwrap(),
            ))
        } else {
            self.wifi_ip = String::try_from(ip).unwrap();
            Ok(())
        }
    }

    fn show(&mut self) -> DisplayResult<()> {
        let mut current_string = heapless::String::<64>::new();
        write!(&mut current_string, "Current: {} mA", self.current_value)
            .map_err(|_| DisplayError::TextRenderError)?;
        let mut voltage_text = heapless::String::<64>::new();
        write!(&mut voltage_text, "Voltage: {:.2} V", self.battery_voltage)
            .map_err(|_| DisplayError::TextRenderError)?;
        let mut battery_text = heapless::String::<64>::new();
        write!(&mut battery_text, "Battery: {}%", self.battery_percentage)
            .map_err(|_| DisplayError::TextRenderError)?;
        let mut wifi_string = heapless::String::<64>::new();
        write!(&mut wifi_string, "WiFi: {}", self.wifi_ip)
            .map_err(|_| DisplayError::TextRenderError)?;
        let mut memory_text = heapless::String::<64>::new();
        write!(&mut memory_text, "Mem: {:.1} kB / {:.1} kB", self.free_memory, self.total_memory)
            .map_err(|_| DisplayError::TextRenderError)?;
        let mut robot_id_text = heapless::String::<32>::new();
        write!(&mut robot_id_text, "ID: {}", self.device_id)
            .map_err(|_| DisplayError::TextRenderError)?;

        Text::new(&current_string, Point::new(10, 20), self.text_style)
            .draw(&mut self.display)
            .unwrap();
        Text::new(&voltage_text, Point::new(10, 40), self.text_style)
            .draw(&mut self.display)
            .unwrap();
        Text::new(&battery_text, Point::new(10, 60), self.text_style)
            .draw(&mut self.display)
            .unwrap();
        Text::new(&wifi_string, Point::new(10, 80), self.text_style)
            .draw(&mut self.display)
            .unwrap();
        // Text::new(&memory_text, Point::new(10, 100), self.text_style)
        //     .draw(&mut self.display)
        //     .unwrap();
        let large_text_style = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
        Text::with_alignment(&robot_id_text, Point::new(120, 125), large_text_style,Alignment::Center)
            .draw(&mut self.display)
            .unwrap();

        Circle::new(Point::new(210, 10), 20)
            .into_styled(if self.mqtt_connected {
                PrimitiveStyle::with_fill(Rgb565::GREEN)
            } else {
                PrimitiveStyle::with_fill(Rgb565::RED)
            })
            .draw(&mut self.display)
            .unwrap();
        Ok(())
    }

    fn write_mqtt_status(&mut self, connected: bool) -> DisplayResult<()> {
        self.mqtt_connected = connected;
        Ok(())
    }
    
    fn write_memory_stats(&mut self, free_memory: f32, total_memory: f32) -> DisplayResult<()> {
        self.free_memory = free_memory;
        self.total_memory = total_memory;
        Ok(())
    }
}
