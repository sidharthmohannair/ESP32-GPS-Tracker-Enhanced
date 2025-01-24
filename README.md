# ESP32 GPS Tracker Enhanced

![License](https://img.shields.io/badge/license-MIT-blue)
![Platform](https://img.shields.io/badge/platform-ESP32-brightgreen)
![GPS](https://img.shields.io/badge/GPS-uBlox-yellow)

![GPS coneected to ESP32 outdoor test setup]()

## Overview

ESP32 GPS Tracker Enhanced is a professional-grade GPS tracking firmware tailored for ESP32. This firmware features advanced NMEA parsing, checksum validation, and intelligent error handling, making it a reliable solution for applications such as GPS antenna tracking, UAV navigation, and geolocation systems.

---

## Features

- **Advanced NMEA Parsing**: Efficiently processes GGA and RMC sentences for precise GPS data extraction.
- **Checksum Validation**: Ensures NMEA sentence integrity to prevent erroneous data.
- **GPS Fix Verification**: Considers satellite count, HDOP, and fix quality for accurate fixes.
- **Millis Roll-Over Management**: Prevents runtime errors in extended operations.
- **Module Reset Functionality**: Resets the GPS module automatically after prolonged inactivity.
- **Improved Error Handling**: Detects and manages timeouts, incomplete sentences, and other anomalies.
- **Flexible Time Formatting**: Outputs UTC time and date in a user-friendly format.

---

## Getting Started

### Prerequisites

#### Hardware
- ESP32 microcontroller
- uBlox GPS module (or any NMEA-compatible GPS receiver)

#### Software
- Arduino IDE or PlatformIO
- ESP32 board package installed
- Required libraries pre-installed

### Wiring Setup

| GPS Module Pin | ESP32 Pin |
|----------------|-----------|
| TX             | GPIO 17   |
| RX             | GPIO 16   |
| GND            | GND       |
| VCC            | 3.3V/5V   |

---

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/sidharthmohannair/ESP32-GPS-Tracker-Enhanced.git
   cd ESP32-GPS-Tracker-Enhanced
   ```

2. Open the project in your preferred IDE (e.g., Arduino IDE).

3. Upload the firmware to the ESP32.

4. Monitor the serial output at 115200 baud for real-time GPS data.

---

## Configuration

| Parameter             | Default Value | Description                          |
|-----------------------|---------------|--------------------------------------|
| `GPS_RX`              | 17            | ESP32 pin connected to GPS TX       |
| `GPS_TX`              | 16            | ESP32 pin connected to GPS RX       |
| `GPS_TIMEOUT`         | 5000 ms       | Time to warn about GPS inactivity   |
| `GPS_RESET_TIMEOUT`   | 10000 ms      | Time to reset the GPS module        |
| `KNOTS_TO_MS`         | 0.514444      | Conversion factor for speed in m/s  |

---

## Example Output

```plaintext
=== GPS Tracker Data ===
Latitude: 28.704060°
Longitude: 77.102493°
Altitude: 216.0 m
GPS Heading: 45.1°
Fix Quality: 1
Fix Status: A
Satellites: 7
HDOP: 1.0
Ground Speed: 2.00 m/s
Magnetic Variation: 1.1°
Time (UTC): 12:34:56
Date: 22/01/25
```

---

## Roadmap

- Expand NMEA sentence support (e.g., GSA, GSV).
- Add SD card logging for offline data analysis.
- Integrate Bluetooth/Wi-Fi for real-time data streaming.

---

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a feature branch (`git checkout -b feature-name`).
3. Commit your changes and push to your fork.
4. Open a pull request with a clear description of your changes.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

