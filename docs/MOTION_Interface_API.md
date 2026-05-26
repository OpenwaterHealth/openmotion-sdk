# MOTIONInterface API Reference

**Module:** `omotion.Interface`  
**Class:** `MOTIONInterface`

**Purpose:** High-level façade that integrates Console and Sensor modules, providing unified access to both console device and left/right sensor endpoints. Manages USB connections, device monitoring, and provides convenience methods for complex camera workflows including histogram capture with automatic FPGA programming and sensor configuration.

---

## Table of Contents

- [Constructor](#constructor)
- [Attributes](#attributes)
- [Connection Management](#connection-management)
- [USB Device Monitoring](#usb-device-monitoring)
- [Multi-Sensor Operations](#multi-sensor-operations)
- [Camera Histogram Workflow](#camera-histogram-workflow)
- [Utility Methods](#utility-methods)
- [Static Factory Methods](#static-factory-methods)
- [Signal Handling](#signal-handling)
- [Usage Examples](#usage-examples)

---

## Constructor

### `__init__(vid: int = 0x0483, sensor_pid: int = SENSOR_MODULE_PID, console_pid: int = CONSOLE_MODULE_PID, baudrate: int = 921600, timeout: int = 30, run_async: bool = False, demo_mode: bool = False)`

Initialize the MOTIONInterface with console and sensor modules.

**Parameters:**
- `vid` (int): Vendor ID for USB devices (default: 0x0483)
- `sensor_pid` (int): Product ID for sensor modules (default: SENSOR_MODULE_PID)
- `console_pid` (int): Product ID for console module (default: CONSOLE_MODULE_PID)
- `baudrate` (int): UART baud rate (default: 921600)
- `timeout` (int): Communication timeout in seconds (default: 30)
- `run_async` (bool): Enable asynchronous mode (default: False)
- `demo_mode` (bool): Enable demo mode for testing without hardware (default: False)

**Notes:**
- Constructs the three stable device handles: `console` (`MotionConsole`),
  `left` and `right` (`MotionSensor`).
- Starts a single `ConnectionMonitor` daemon thread that drives the
  state machines on all three handles in response to OS hotplug events
  + a 200 ms poll sweep.
- Any already-connected devices are picked up by the monitor's first poll
  after `.start()`; the handles transition to `CONNECTED` automatically.

**Example:**
```python
# Basic initialization with defaults
interface = MOTIONInterface()

# Custom configuration
interface = MOTIONInterface(
    vid=0x0483,
    sensor_pid=0x5740,
    console_pid=0x5741,
    baudrate=921600,
    timeout=30,
    run_async=False
)
```

---

## Attributes

### `console_module: MOTIONConsole`

The console device wrapper providing access to controller/console MCU operations.

**Access:**
```python
interface.console_module.get_version()
interface.console_module.get_fan_speed(fan_index=1)
```

---

### `sensors: dict[str, MOTIONSensor | None]`

Dictionary containing left and right sensor instances.

**Structure:**
```python
{
    "left": MOTIONSensor | None,
    "right": MOTIONSensor | None
}
```

**Access:**
```python
# Access specific sensor
left_sensor = interface.sensors["left"]
if left_sensor:
    temp = left_sensor.imu_get_temperature()

# Check both sensors
for side, sensor in interface.sensors.items():
    if sensor and sensor.is_connected():
        print(f"{side} sensor connected")
```

---

## Connection Management

### `is_device_connected() -> tuple[bool, bool, bool]`

Check connection status of all devices.

**Returns:**
- `tuple[bool, bool, bool]`: (console_connected, left_connected, right_connected)

**Example:**
```python
console_ok, left_ok, right_ok = interface.is_device_connected()
print(f"Console: {console_ok}, Left: {left_ok}, Right: {right_ok}")
```

---

### `_initialize_sensors()`

Internal method to initialize MOTIONSensor instances for currently connected devices.

**Notes:**
- Called automatically during construction
- Creates sensor wrappers for any MotionComposite devices already connected
- Not typically called directly by users

---

### `_on_sensor_connect(sensor_id: str, connection_type: str)`

Internal signal handler for sensor connection events.

**Parameters:**
- `sensor_id` (str): Sensor identifier ("SENSOR_LEFT" or "SENSOR_RIGHT")
- `connection_type` (str): Type of connection event

**Notes:**
- Automatically creates MOTIONSensor wrapper when device connects
- Forwards signal to external listeners
- Not typically called directly by users

---

### `_on_sensor_disconnect(sensor_id: str, connection_type: str)`

Internal signal handler for sensor disconnection events.

**Parameters:**
- `sensor_id` (str): Sensor identifier ("SENSOR_LEFT" or "SENSOR_RIGHT")
- `connection_type` (str): Type of disconnection event

**Notes:**
- Clears MOTIONSensor reference when device disconnects
- Forwards signal to external listeners
- Not typically called directly by users

---

## USB Device Monitoring

### `async start_monitoring(interval: int = 1) -> None`

Start monitoring for USB device connections (async mode only).

**Parameters:**
- `interval` (int): Polling interval in seconds (default: 1)

**Notes:**
- Only available in async mode (run_async=True)
- Monitors both console and sensor devices
- Uses asyncio.gather to run monitors concurrently
- Must be awaited

**Example:**
```python
import asyncio

async def main():
    interface = MOTIONInterface(run_async=True)
    await interface.start_monitoring(interval=1)

asyncio.run(main())
```

---

### `stop_monitoring() -> None`

Stop monitoring for USB device connections.

**Notes:**
- Safe to call even if monitoring not started
- Stops monitoring for console and both sensor devices
- Handles cases where stop_monitoring method may not exist

**Example:**
```python
interface.stop_monitoring()
```

---

## Multi-Sensor Operations

### `run_on_sensors(func_name: str, *args, target: str | Iterable[str] | None = None, include_disconnected: bool = True, **kwargs) -> dict[str, Any]`

Run a MOTIONSensor method on selected sensors and return results.

**Parameters:**
- `func_name` (str): Name of the MOTIONSensor method to call
- `*args`: Positional arguments to pass to the method
- `target` (str | Iterable[str] | None): Which sensor(s) to target:
  - `None` (default): Run on all sensors
  - `"left"` or `"right"`: Run only on that sensor
  - `"all"` or `"*"`: Same as None
  - Iterable[str]: e.g., ["left", "right"]
- `include_disconnected` (bool): If True, include keys for disconnected sensors with value None (default: True)
- `**kwargs`: Keyword arguments to pass to the method

**Returns:**
- `dict[str, Any]`: Dictionary mapping sensor name to return value or None

**Notes:**
- Continues execution even if one sensor fails
- Logs errors but doesn't raise exceptions
- Returns None for disconnected sensors if include_disconnected=True

**Examples:**
```python
# Get temperature from all sensors
temps = interface.run_on_sensors("imu_get_temperature")
# Returns: {"left": 35.2, "right": 36.1}

# Ping only left sensor
results = interface.run_on_sensors("ping", target="left")
# Returns: {"left": True}

# Reset cameras on both sensors
results = interface.run_on_sensors(
    "reset_camera_sensor",
    camera_position=0x01,
    target=["left", "right"]
)
# Returns: {"left": True, "right": True}

# Get versions, exclude disconnected
versions = interface.run_on_sensors(
    "get_version",
    include_disconnected=False
)
# Returns: {"left": "v1.2.3"} (if right is disconnected)
```

---

## Camera Histogram Workflow

### `get_camera_histogram(sensor_side: str, camera_id: int, test_pattern_id: int = 4, auto_upload: bool = True) -> tuple[list[int], list[int]] | None`

High-level orchestrated workflow to capture and retrieve a histogram from a specific camera.

**Parameters:**
- `sensor_side` (str): Which sensor module to use ("left" or "right")
- `camera_id` (int): Camera ID (0-7)
- `test_pattern_id` (int): Test pattern to use (0-4, default: 4)
  - 0: 242.83µs
  - 1: 250.67µs
  - 2: 344.67µs
  - 3: 352.50µs
  - 4: 1098.00µs
- `auto_upload` (bool): Automatically program FPGA if needed (default: True)

**Returns:**
- `tuple[list[int], list[int]]`: (histogram_values, hidden_flags)
  - `histogram_values`: List of 1024 integers (24-bit values)
  - `hidden_flags`: List of 1024 bytes (4th byte from each 4-byte group)
- `None`: On any error

**Workflow Steps:**
1. **Status Check**: Verify camera peripheral is READY
2. **FPGA Programming**: Program FPGA if not already configured (if auto_upload=True)
3. **Register Configuration**: Configure camera sensor registers
4. **Test Pattern Setup**: Apply specified test pattern
5. **Verification**: Confirm camera is ready for histogram capture
6. **Capture**: Trigger histogram capture
7. **Retrieval**: Read histogram data (4096 bytes)
8. **Parsing**: Convert bytes to integers and extract hidden flags

**Raises:**
- Logs errors but returns None rather than raising exceptions

**Notes:**
- Validates sensor_side and camera_id parameters
- Checks sensor connection before proceeding
- Uses camera bitmask for operations (1 << camera_id)
- Automatically handles FPGA programming and sensor configuration
- Returns parsed histogram data ready for analysis

**Example:**
```python
# Capture histogram from left sensor, camera 0
result = interface.get_camera_histogram(
    sensor_side="left",
    camera_id=0,
    test_pattern_id=4,
    auto_upload=True
)

if result:
    values, flags = result
    print(f"Captured {len(values)} histogram values")
    print(f"First 10 values: {values[:10]}")
    print(f"First 10 flags: {flags[:10]}")
else:
    print("Histogram capture failed")
```

**Detailed Example with Error Handling:**
```python
# Capture from right sensor with custom test pattern
sensor_side = "right"
camera_id = 3

# Check if sensor is connected first
_, _, right_ok = interface.is_device_connected()
if not right_ok:
    print(f"{sensor_side} sensor not connected")
else:
    result = interface.get_camera_histogram(
        sensor_side=sensor_side,
        camera_id=camera_id,
        test_pattern_id=2,
        auto_upload=True
    )
    
    if result:
        histogram, flags = result
        
        # Analyze histogram
        max_val = max(histogram)
        min_val = min(histogram)
        avg_val = sum(histogram) / len(histogram)
        
        print(f"Histogram Statistics:")
        print(f"  Max: {max_val}")
        print(f"  Min: {min_val}")
        print(f"  Avg: {avg_val:.2f}")
        
        # Check for specific flag patterns
        if any(f != 0 for f in flags):
            print("Warning: Non-zero flags detected")
    else:
        print("Failed to capture histogram")
```

---

## Utility Methods

### `@staticmethod bytes_to_integers(byte_array: bytes) -> tuple[list[int], list[int]]`

Parse a 4096-byte histogram into integer values and hidden flags.

**Parameters:**
- `byte_array` (bytes): Exactly 4096 bytes of histogram data

**Returns:**
- `tuple[list[int], list[int]]`: (integers, hidden_figures)
  - `integers`: List of 1024 integers (24-bit little-endian values from bytes 0-2)
  - `hidden_figures`: List of 1024 bytes (4th byte from each 4-byte group)

**Raises:**
- `ValueError`: If byte_array is not exactly 4096 bytes

**Notes:**
- Processes data in 4-byte chunks
- Extracts 24-bit little-endian integer from first 3 bytes
- Collects 4th byte as "hidden figure" (possibly status/flag data)

**Example:**
```python
# Typical usage (called internally by get_camera_histogram)
raw_data = b'\x00' * 4096  # Example: 4096 bytes of data
values, flags = MOTIONInterface.bytes_to_integers(raw_data)

print(f"Parsed {len(values)} values and {len(flags)} flags")
```

---

### `@staticmethod get_sdk_version() -> str`

Get the SDK version string.

**Returns:**
- `str`: Version string (e.g., "1.2.3")

**Example:**
```python
version = MOTIONInterface.get_sdk_version()
print(f"Open-Motion SDK version: {version}")
```

---

## Static Factory Methods

### `@staticmethod acquire_motion_interface() -> tuple[MOTIONInterface, bool, bool, bool]`

Convenience factory method to create interface and check device connections.

**Returns:**
- `tuple`: (interface, console_connected, left_connected, right_connected)
  - `interface`: MOTIONInterface instance
  - `console_connected`: Console device connection status
  - `left_connected`: Left sensor connection status
  - `right_connected`: Right sensor connection status

**Notes:**
- Creates interface with default parameters
- Immediately checks connection status
- Convenient for quick setup and validation

**Example:**
```python
# Quick setup with connection check
interface, console_ok, left_ok, right_ok = MOTIONInterface.acquire_motion_interface()

if console_ok:
    print("Console connected")
    version = interface.console_module.get_version()
    print(f"Console version: {version}")

if left_ok:
    print("Left sensor connected")
    temp = interface.sensors["left"].imu_get_temperature()
    print(f"Left sensor temp: {temp}°C")

if right_ok:
    print("Right sensor connected")
    temp = interface.sensors["right"].imu_get_temperature()
    print(f"Right sensor temp: {temp}°C")
```

---

## Signal Handling

### Signal Architecture

MOTIONInterface inherits from `SignalWrapper` and provides Qt-style signals for device events.

**Available Signals:**
- `signal_connect`: Emitted when a device connects
- `signal_disconnect`: Emitted when a device disconnects
- `signal_data_received`: Emitted when data is received

**Notes:**
- Requires PyQt5 or PyQt6 for full signal functionality
- Falls back to basic implementation if Qt not available
- Signals from console UART and sensor composites are forwarded to interface
- Useful for building GUI applications with real-time device monitoring

**Example with PyQt:**
```python
from PyQt5.QtWidgets import QApplication
import sys

class DeviceMonitor:
    def __init__(self):
        self.interface = MOTIONInterface(run_async=False)
        
        # Connect signals
        self.interface.signal_connect.connect(self.on_device_connect)
        self.interface.signal_disconnect.connect(self.on_device_disconnect)
        self.interface.signal_data_received.connect(self.on_data_received)
    
    def on_device_connect(self, device_id, connection_type):
        print(f"Device connected: {device_id} ({connection_type})")
    
    def on_device_disconnect(self, device_id, connection_type):
        print(f"Device disconnected: {device_id} ({connection_type})")
    
    def on_data_received(self, data):
        print(f"Data received: {len(data)} bytes")

# Usage
app = QApplication(sys.argv)
monitor = DeviceMonitor()
app.exec_()
```

---

## Usage Examples

### Basic Setup and Connection Check

```python
from omotion.Interface import MOTIONInterface

# Create interface
interface = MOTIONInterface()

# Check device connections
console_ok, left_ok, right_ok = interface.is_device_connected()

print(f"Console: {'Connected' if console_ok else 'Not connected'}")
print(f"Left Sensor: {'Connected' if left_ok else 'Not connected'}")
print(f"Right Sensor: {'Connected' if right_ok else 'Not connected'}")
```

---

### Using the Factory Method

```python
from omotion.Interface import MOTIONInterface

# Quick setup with built-in connection check
interface, console_ok, left_ok, right_ok = MOTIONInterface.acquire_motion_interface()

if not console_ok:
    print("Warning: Console not connected")

if left_ok and right_ok:
    print("Both sensors ready!")
elif left_ok:
    print("Only left sensor available")
elif right_ok:
    print("Only right sensor available")
else:
    print("No sensors connected")
```

---

### Console Operations

```python
# Access console module
console = interface.console_module

# Get device information
version = console.get_version()
hw_id = console.get_hardware_id()
print(f"Console version: {version}, HW ID: {hw_id}")

# Read fan PWM feedback (read-only, fan_index 1..3, ~50 ms per call)
duty = console.get_fan_speed(fan_index=1)
print(f"Fan 1 duty: {duty}%")

# Set LED
console.set_rgb_led(2)  # Green

# Get temperatures
mcu_temp, safety_temp, ta_temp = console.get_temperatures()
print(f"Temperatures - MCU: {mcu_temp}°C, Safety: {safety_temp}°C")
```

---

### Direct Sensor Access

```python
# Access specific sensor
left_sensor = interface.sensors["left"]

if left_sensor and left_sensor.is_connected():
    # Get version
    version = left_sensor.get_version()
    print(f"Left sensor version: {version}")
    
    # Read IMU
    temp = left_sensor.imu_get_temperature()
    accel = left_sensor.imu_get_accelerometer()
    gyro = left_sensor.imu_get_gyroscope()
    
    print(f"Temperature: {temp}°C")
    print(f"Accelerometer: {accel}")
    print(f"Gyroscope: {gyro}")
    
    # Camera operations
    left_sensor.reset_camera_sensor(0x01)  # Reset camera 0
    left_sensor.enable_camera_fpga(0x01)   # Enable FPGA
```

---

### Multi-Sensor Operations

```python
# Ping all sensors
results = interface.run_on_sensors("ping")
print(f"Ping results: {results}")

# Get versions from all connected sensors
versions = interface.run_on_sensors("get_version")
for side, version in versions.items():
    if version:
        print(f"{side.capitalize()} sensor version: {version}")

# Get temperature from both sensors
temps = interface.run_on_sensors("imu_get_temperature")
for side, temp in temps.items():
    if temp is not None:
        print(f"{side.capitalize()} sensor temperature: {temp}°C")

# Reset camera 0 on left sensor only
results = interface.run_on_sensors(
    "reset_camera_sensor",
    camera_position=0x01,
    target="left"
)

# Enable all cameras on both sensors
results = interface.run_on_sensors(
    "enable_camera_fpga",
    camera_position=0xFF,
    target=["left", "right"]
)
```

---

### Histogram Capture Workflow

```python
# Simple histogram capture
result = interface.get_camera_histogram(
    sensor_side="left",
    camera_id=0,
    test_pattern_id=4
)

if result:
    values, flags = result
    print(f"Captured {len(values)} histogram values")
    
    # Basic statistics
    print(f"Max value: {max(values)}")
    print(f"Min value: {min(values)}")
    print(f"Average: {sum(values) / len(values):.2f}")
```

---

### Advanced Histogram Analysis

```python
import matplotlib.pyplot as plt
import numpy as np

def capture_and_plot_histogram(interface, sensor_side, camera_id):
    """Capture histogram and create visualization."""
    
    result = interface.get_camera_histogram(
        sensor_side=sensor_side,
        camera_id=camera_id,
        test_pattern_id=4,
        auto_upload=True
    )
    
    if not result:
        print("Failed to capture histogram")
        return
    
    values, flags = result
    
    # Convert to numpy array for analysis
    hist_array = np.array(values)
    
    # Statistics
    print(f"Histogram Statistics:")
    print(f"  Count: {len(hist_array)}")
    print(f"  Mean: {np.mean(hist_array):.2f}")
    print(f"  Std Dev: {np.std(hist_array):.2f}")
    print(f"  Min: {np.min(hist_array)}")
    print(f"  Max: {np.max(hist_array)}")
    
    # Plot histogram
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.plot(hist_array)
    plt.title(f'{sensor_side.capitalize()} Sensor - Camera {camera_id}')
    plt.xlabel('Bin')
    plt.ylabel('Value')
    plt.grid(True)
    
    plt.subplot(1, 2, 2)
    plt.hist(hist_array, bins=50)
    plt.title('Value Distribution')
    plt.xlabel('Value')
    plt.ylabel('Frequency')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

# Usage
interface, console_ok, left_ok, right_ok = MOTIONInterface.acquire_motion_interface()

if left_ok:
    capture_and_plot_histogram(interface, "left", 0)
```

---

### Async Monitoring Example

```python
import asyncio
from omotion.Interface import MOTIONInterface

async def monitor_devices():
    """Monitor devices asynchronously."""
    interface = MOTIONInterface(run_async=True)
    
    # Start monitoring
    monitoring_task = asyncio.create_task(
        interface.start_monitoring(interval=1)
    )
    
    try:
        # Do work while monitoring
        for i in range(10):
            console_ok, left_ok, right_ok = interface.is_device_connected()
            print(f"Iteration {i}: Console={console_ok}, Left={left_ok}, Right={right_ok}")
            await asyncio.sleep(2)
    finally:
        # Stop monitoring
        interface.stop_monitoring()
        monitoring_task.cancel()
        try:
            await monitoring_task
        except asyncio.CancelledError:
            pass

# Run
asyncio.run(monitor_devices())
```

---

### Complete Application Example

```python
from omotion.Interface import MOTIONInterface
import time

def main():
    """Complete workflow demonstrating interface capabilities."""
    
    # Initialize interface
    print("Initializing MOTIONInterface...")
    interface, console_ok, left_ok, right_ok = MOTIONInterface.acquire_motion_interface()
    
    # Check SDK version
    sdk_version = MOTIONInterface.get_sdk_version()
    print(f"SDK Version: {sdk_version}")
    
    # Verify connections
    print(f"\nDevice Status:")
    print(f"  Console: {'✓' if console_ok else '✗'}")
    print(f"  Left Sensor: {'✓' if left_ok else '✗'}")
    print(f"  Right Sensor: {'✓' if right_ok else '✗'}")
    
    if not (console_ok and left_ok):
        print("\nRequired devices not connected. Exiting.")
        return
    
    # Console operations
    print("\n--- Console Operations ---")
    console_version = interface.console_module.get_version()
    print(f"Console Firmware: {console_version}")
    
    # Read fan 1 PWM feedback (read-only, ~50 ms)
    duty = interface.console_module.get_fan_speed(fan_index=1)
    print(f"Fan 1 duty: {duty}%")
    
    # Set LED to green
    interface.console_module.set_rgb_led(2)
    print("LED set to green")
    
    # Get temperatures
    mcu, safety, ta = interface.console_module.get_temperatures()
    print(f"Temperatures: MCU={mcu:.1f}°C, Safety={safety:.1f}°C, TA={ta:.1f}°C")
    
    # Sensor operations
    print("\n--- Sensor Operations ---")
    
    # Get versions from all sensors
    versions = interface.run_on_sensors("get_version")
    for side, version in versions.items():
        if version:
            print(f"{side.capitalize()} sensor firmware: {version}")
    
    # Get IMU data from all sensors
    temps = interface.run_on_sensors("imu_get_temperature")
    for side, temp in temps.items():
        if temp is not None:
            print(f"{side.capitalize()} sensor temperature: {temp:.1f}°C")
    
    # Camera histogram capture
    print("\n--- Camera Histogram Capture ---")
    print("Capturing histogram from left sensor, camera 0...")
    
    result = interface.get_camera_histogram(
        sensor_side="left",
        camera_id=0,
        test_pattern_id=4,
        auto_upload=True
    )
    
    if result:
        values, flags = result
        print(f"✓ Histogram captured successfully")
        print(f"  Values: {len(values)} bins")
        print(f"  Range: {min(values)} - {max(values)}")
        print(f"  Average: {sum(values) / len(values):.2f}")
        print(f"  First 10 values: {values[:10]}")
    else:
        print("✗ Histogram capture failed")
    
    # Cleanup
    print("\n--- Cleanup ---")
    interface.console_module.set_rgb_led(0)  # Turn off LED
    print("LED turned off")
    
    print("\nDone!")

if __name__ == "__main__":
    main()
```

---

## Notes

- **Thread Safety**: Interface is not inherently thread-safe; use appropriate locking if accessing from multiple threads
- **Async Mode**: When `run_async=True`, use `await` for monitoring methods
- **Signal Support**: Full signal functionality requires PyQt5 or PyQt6
- **Error Handling**: Most methods log errors rather than raising exceptions; check return values
- **Resource Cleanup**: Always disconnect when done or use context managers if implemented
- **Demo Mode**: Use `demo_mode=True` for testing without hardware
- **Camera IDs**: Valid camera IDs are 0-7 (8 cameras per sensor module)
- **Histogram Data**: Returns 1024 values from 4096 bytes (4 bytes per value)
- **FPGA Programming**: Automatic FPGA upload can take several seconds
- **Baud Rate**: Default 921600 works for most configurations; adjust if experiencing communication issues
- **Multi-Sensor Operations**: Use `run_on_sensors()` for efficient batch operations across sensors

