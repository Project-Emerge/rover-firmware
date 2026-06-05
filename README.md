# Project Emerge Firmware

ESP32-C3 firmware for a robotic platform with MQTT communication, motor control, battery monitoring, and display management.

## Features

- 🔋 **Battery Management**: Real-time voltage and current monitoring with INA219 sensor
- 🚗 **Motor Control**: Differential drive control for 4-wheel platforms using TB6612FNG drivers
- 📱 **Display**: ST7789 TFT display for status visualization
- 📡 **Wireless**: WiFi connectivity with MQTT communication protocol
- ⚡ **Embassy**: Async/await based firmware using Embassy framework
- 🔧 **Configurable**: Easy configuration via environment variables

## Prerequisites

### Hardware Requirements

- **ESP32-C3 development board**
- **ST7789 display** (135x240 pixels)
- **TB6612FNG motor drivers** (2x for 4-wheel drive)
- **INA219 current/voltage sensor** for battery monitoring
- **Battery pack** (2-cell Li-ion/Li-Po, 7.4V nominal)

### Software Requirements

- **[Rust](https://rustlang.org/tools/install)** (latest stable version)
- **[probe-rs](https://probe.rs/docs/getting-started/installation/)** for flashing and debugging
- **ESP Rust toolchain** with RISC-V target support

### Hardware Pin Configuration

| Component | ESP32-C3 GPIO | Notes |
|-----------|---------------|--------|
| **I2C SDA** | Board-specific | For INA219 sensor |
| **I2C SCL** | Board-specific | For INA219 sensor |
| **ST7789 Display** | SPI pins | CS, DC, RST pins (check your board) |
| **Motor Drivers** | PWM pins | TB6612FNG control pins |

## Setup

### 1. Install ESP Rust Toolchain

```bash
# Install the ESP Rust toolchain
cargo install espup
espup install
# Source the environment (add to your shell profile)
. $HOME/export-esp.sh
```

### 2. Install probe-rs

```bash
yay -S probe-rs
```

Or

```bash
# Install probe-rs for flashing
curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
```

### 3. Configure Environment Variables

The `.cargo/config.toml` file in the project root contains the configuration for the build environment.
Update it with your specific settings:

```toml
[target.riscv32imc-unknown-none-elf]
runner = "probe-rs run --chip=esp32c3 --preverify --always-print-stacktrace --no-location --catch-hardfault"

[env]
DEFMT_LOG = "info"                 # Log level: trace, debug, info, warn, error
SSID = "your_wifi_ssid"           # Your WiFi network name
PASSWORD = "your_wifi_password"    # Your WiFi password
MQTT_BROKER = "192.168.1.100"     # MQTT broker IP address
MQTT_PORT = "1883"                # MQTT broker port
ROBOT_ID = "0"                    # Unique robot identifier (0-255)

[build]
target = "riscv32imc-unknown-none-elf"

[unstable]
build-std = ["alloc", "core"]
```

**⚠️ Security Note**: Never commit sensitive credentials to version control! Consider using environment variables for production:

```bash
# Set environment variables in your shell profile (.bashrc, .zshrc, etc.)
export WIFI_SSID="your_wifi_ssid"
export WIFI_PASSWORD="your_wifi_password"
export MQTT_BROKER="192.168.1.100"
```

Then build the project.

## Building

### Quick Start

```bash
# Build and flash in one command (development)
cargo run

# Build and flash optimized version
cargo run --release
```

### Build Commands

#### Development Build (with debug info)

```bash
cargo build
```

#### Release Build (optimized for performance)

```bash
cargo build --release
```

#### Code Quality Checks

```bash
# Run clippy for linting
cargo clippy --all-features --workspace -- -D warnings

# Format code
cargo fmt --all

# Check formatting without applying
cargo fmt --all -- --check
```

## Flashing

### Standard Flashing (Recommended)

```bash
# Flash development build
cargo run

# Flash release build (optimized)
cargo run --release
```

### Manual Flashing

If you need more control over the flashing process:

```bash
# Flash with specific chip target
probe-rs run --chip=esp32c3 target/riscv32imc-unknown-none-elf/release/project-emerge-firmware

# Flash with different speed (if having connection issues)
probe-rs run --chip=esp32c3 --speed 115200 target/riscv32imc-unknown-none-elf/debug/project-emerge-firmware
```

### Monitoring

```bash
# Monitor RTT/defmt output
probe-rs run --chip=esp32c3 --catch-hardfault --rtt-scan-memory target/riscv32imc-unknown-none-elf/debug/project-emerge-firmware

# Alternative monitoring
probe-rs attach --chip=esp32c3 --rtt-scan-memory target/riscv32imc-unknown-none-elf/debug/project-emerge-firmware
```

## Testing

### Unit Tests

```bash
# Run unit tests on host
cargo test
```

### Hardware-in-the-Loop Tests

```bash
# Flash and run embedded tests on actual hardware
cargo test --target riscv32imc-unknown-none-elf

# Run specific test
cargo test --target riscv32imc-unknown-none-elf -- --exact test_name
```

## MQTT Communication

The firmware uses MQTT for wireless communication. Replace `{ROBOT_ID}` with your robot's unique identifier.

### Subscribed Topics (Commands to Robot)

| Topic | Description | Payload Example |
|-------|-------------|-----------------|
| `robot/{ROBOT_ID}/move` | Motor movement commands | `{"left": 0.5, "right": -0.3}` |
| `robot/{ROBOT_ID}/stop` | Emergency stop | `{}` |
| `robot/{ROBOT_ID}/config` | Configuration updates | `{"max_speed": 0.8}` |

#### Movement Command Details

```json
{
  "left": 0.5,   // Left wheel speed: -1.0 (full reverse) to 1.0 (full forward)
  "right": -0.3  // Right wheel speed: -1.0 (full reverse) to 1.0 (full forward)
}
```

### Published Topics (Status from Robot)

| Topic | Description | Update Frequency |
|-------|-------------|-----------------|
| `robot/{ROBOT_ID}/battery` | Battery status | Every 5 seconds |
| `robot/{ROBOT_ID}/status` | General robot status | On state change |
| `robot/{ROBOT_ID}/heartbeat` | Connection health | Every 30 seconds |

#### Battery Status Details

```json
{
  "battery_voltage": 7.8,      // Voltage in V (5.0-8.4V range)
  "battery_percentage": 85,    // Percentage (0-100%)
  "absorbed_current": 150,     // Current draw in mA
  "low_battery_warning": false // Warning flag
}
```

## Configuration

### Robot ID

Each robot must have a unique ID set in the `ROBOT_ID` environment variable. This is used for:

## Configuration

### Robot ID

Each robot must have a unique ID (0-255) set in the `ROBOT_ID` environment variable. This ID is used for:

- 🏷️ **MQTT topic namespacing** - Ensures messages are routed correctly
- 📱 **Display identification** - Shows robot ID on the display
- 🌐 **Network differentiation** - Distinguishes robots on the same network

### Motor Control Parameters

The firmware supports differential drive control:

| Parameter | Range | Description |
|-----------|-------|-------------|
| `left` | -1.0 to 1.0 | Left wheel speed (negative = reverse) |
| `right` | -1.0 to 1.0 | Right wheel speed (negative = reverse) |

**Movement Examples:**

- Forward: `{"left": 0.5, "right": 0.5}`
- Backward: `{"left": -0.5, "right": -0.5}`
- Turn left: `{"left": -0.3, "right": 0.3}`
- Turn right: `{"left": 0.3, "right": -0.3}`

### Battery Monitoring System

- **Battery type**: 2-cell Li-ion/Li-Po (7.4V nominal)
- **Voltage range**: 5.0V (0%) to 8.4V (100%)
- **Sensor**: INA219 for current and voltage measurement
- **Features**:
  - Real-time voltage monitoring
  - Current consumption tracking
  - Automatic low battery warnings
  - Battery percentage calculation

## Troubleshooting

### Build Issues

```bash
# Clean build artifacts and start fresh
cargo clean

# Update all dependencies
cargo update

# Check your Rust toolchain
rustup show

# Reinstall ESP toolchain if needed
espup install --force
```

### Flash Issues

```bash
# List connected devices
probe-rs list

# Check if ESP32-C3 is detected
lsusb | grep -i esp

# Try flashing with different speed
probe-rs run --chip=esp32c3 --speed 115200 target/riscv32imc-unknown-none-elf/debug/project-emerge-firmware

# Reset the device manually before flashing
probe-rs erase --chip=esp32c3
```

### Runtime Issues

```bash
# Check RTT/defmt output with crash detection
probe-rs run --chip=esp32c3 --catch-hardfault --rtt-scan-memory target/riscv32imc-unknown-none-elf/debug/project-emerge-firmware

# Monitor logs via RTT (Real-Time Transfer) 
probe-rs attach --chip=esp32c3 --rtt-scan-memory target/riscv32imc-unknown-none-elf/debug/project-emerge-firmware

# Check WiFi connection status
# (Look for "WiFi connected" in the logs)
```

**Common Runtime Checks:**

- ✅ Verify WiFi credentials in config
- ✅ Confirm MQTT broker is reachable
- ✅ Check I2C connections for sensors
- ✅ Ensure adequate power supply

### Common Error Messages

| Error Message | Likely Cause | Solution |
|---------------|--------------|----------|
| `Failed to initialize INA219` | I2C connection issue | Check SDA=GPIO39, SCL=GPIO38 connections |
| `MQTT disconnected` | Network/broker issue | Verify broker IP and network connectivity |
| `WiFi connection failed` | Wrong credentials | Double-check SSID and password |
| `Low memory warning` | Heap exhaustion | Reduce heap usage or increase allocation |
| `Watchdog timeout` | Task blocking | Check for long-running operations |

## Development

### Project Structure

```
src/
├── bin/main.rs           # Main application entry point
├── lib.rs               # Library root
├── robot/               # Robot control modules
│   ├── battery_manager.rs
│   ├── display_manager.rs
│   ├── event_loop.rs
│   ├── events.rs
│   ├── motors_manager.rs
│   └── mod.rs
└── utils/               # Utility modules
    ├── channels.rs
    ├── mqtt_manager.rs
    ├── protocol.rs
    ├── topics.rs
    └── mod.rs
```

### Adding Features

1. Define events in [`robot/events.rs`](src/robot/events.rs)
2. Handle events in [`robot/event_loop.rs`](src/robot/event_loop.rs)
3. Add MQTT topics in [`utils/topics.rs`](src/utils/topics.rs)
4. Update protocol in [`utils/protocol.rs`](src/utils/protocol.rs)

## License

[Add your license information here]

## Contributing

[Add contribution guidelines here]
