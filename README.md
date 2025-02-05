# Sensirion SVM30 Air Quality Sensor Example for ESP-IDF

This example demonstrates how to use the **Sensirion SVM30** air quality sensor with **Espressif ESP32 series** chips using **ESP-IDF**. The **SVM30** provides measurements of **Volatile Organic Compounds (VOC), Nitrogen Oxides (NOx), and ambient temperature/humidity** (if supported by hardware). This project initializes the sensor, reads data periodically, and outputs it via UART.

## Supported Targets

| ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |
| ✅     | ⚠️       | ✅        | ⚠️       | ⚠️       | ⚠️       | ✅        | ✅        |
=======
Sensirion SVM30 Air Quality Sensor Example for ESP-IDF

This example demonstrates how to use the Sensirion SVM30 air quality sensor with Espressif ESP32 series chips using ESP-IDF. The SVM30 provides measurements of Volatile Organic Compounds (VOC), Nitrogen Oxides (NOx), and ambient temperature/humidity (if supported by hardware). This project initializes the sensor, reads data periodically, and outputs it via UART.

| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

Note: This example is primarily tested on ESP32. Other targets may require hardware configuration adjustments.

Features

Initialization of SVM30 sensor

Periodic reading of VOC and CO2 indices

Temperature and humidity measurement (if supported by hardware)

UART-based output for easy monitoring

Hardware Requirements

ESP32 development board (e.g., ESP32-WROOM-32)

Sensirion SVM30 sensor

Breadboard and jumper wires

USB cable for power and programming

Wiring Guide

SVM30 Pin	ESP32 Pin	Description
VCC	3.3V	Power supply (3.3V)
GND	GND	Ground
SDA	GPIO21	I2C Data
SCL	GPIO22	I2C Clock
SEL	GND	I2C mode selection (0V)
Note: Ensure the SEL pin is connected to GND to enable I2C communication.

Getting Started

Prerequisites

ESP-IDF v4.4 or newer installed

Basic understanding of ESP-IDF project structure

Hardware setup as per wiring guide

Installation

Clone the Repository

bash
git clone https://github.com/MouNir9944/Sensirion_SVM30_esp32_idf.git
cd Sensirion_SVM30_esp32_idf
Set Target Chip (Replace esp32 with your target if different)

bash

idf.py set-target esp32
Configure Project (Optional)

bash

idf.py menuconfig
Adjust I2C pin configuration under Example Configuration if needed.

Build and Flash

bash

idf.py build flash monitor
Observe Output
After flashing, the ESP32 will initialize the sensor and start printing measurements:

I (324) example: SVM30 Initialized successfully
I (334) example: VOC Index: 145 ppm, CO2_eq Index: 23 ppm
I (4334) example: T Index: 152 °C, H Index: 25 %
...
Configuration Options

I2C Settings

Modify main/main.c to change default I2C settings:

```c
#define I2C_MASTER_SDA_IO         21      /* GPIO number for SDA */
#define I2C_MASTER_SCL_IO         22      /* GPIO number for SCL */
#define I2C_MASTER_FREQ_HZ        100000  /* I2C master clock frequency */
```
Measurement Interval
Adjust the sampling interval by changing SAMPLE_DELAY_MS in main/main.c:

```c
#define SAMPLE_DELAY_MS 4000  /* 4-second interval */
```
Troubleshooting

Sensor Not Detected

Verify wiring connections

Check I2C address with i2c_scanner example

Ensure SEL pin is grounded for I2C mode


> **Note:** This example is primarily tested on **ESP32**. Other targets may require hardware configuration adjustments.

---

## Features

- **Initialization of SVM30 sensor**
- **Periodic reading of VOC and CO2 indices**
- **Temperature and humidity measurement** (if supported by hardware)
- **UART-based output** for easy monitoring

---

## Hardware Requirements

- **ESP32 development board** (e.g., ESP32-WROOM-32)
- **Sensirion SVM30 sensor**
- **Breadboard and jumper wires**
- **USB cable** for power and programming

### Wiring Guide 

| SVM30 Pin | ESP32 Pin  | Description             |
| --------- | ---------- | ----------------------- |
| **VCC**   | **3.3V**   | Power supply (3.3V)     |
| **GND**   | **GND**    | Ground                  |
| **SDA**   | **GPIO21** | I2C Data                |
| **SCL**   | **GPIO22** | I2C Clock               |
| **SEL**   | **GND**    | I2C mode selection (0V) |

> **Note:** Ensure the **SEL** pin is connected to **GND** to enable I2C communication.

---

## Getting Started

### Prerequisites

- **ESP-IDF v4.4 or newer** installed
- **Basic understanding** of ESP-IDF project structure
- **Hardware setup** as per the wiring guide

### Installation

#### Clone the Repository

```bash
git clone https://github.com/MouNir9944/Sensirion_SVM30_esp32_idf.git
cd Sensirion_SVM30_esp32_idf
```

#### Set Target Chip (Replace `esp32` with your target if different)

```bash
idf.py set-target esp32
```

#### Configure Project (Optional)

```bash
idf.py menuconfig
```

> Adjust **I2C pin configuration** under *Example Configuration* if needed.

#### Build and Flash

```bash
idf.py build flash monitor
```

---

## Observe Output

After flashing, the ESP32 will initialize the sensor and start printing measurements:

```plaintext
I (324) example: SVM30 Initialized successfully
I (334) example: VOC Index: 145 ppm, CO2_eq Index: 23 ppm
I (4334) example: T Index: 152 °C, H Index: 25 %
...
```

---

## Configuration Options

### I2C Settings

Modify `main/main.c` to change default **I2C settings**:

```c
#define I2C_MASTER_SDA_IO         21      /* GPIO number for SDA */
#define I2C_MASTER_SCL_IO         22      /* GPIO number for SCL */
#define I2C_MASTER_FREQ_HZ        100000  /* I2C master clock frequency */
```

### Measurement Interval

Adjust the sampling interval by changing **`SAMPLE_DELAY_MS`** in `main/main.c`:

```c
#define SAMPLE_DELAY_MS 4000  /* 4-second interval */
```

---

## Troubleshooting

### Sensor Not Detected

✅ Verify **wiring connections**
✅ Check **I2C address** with `i2c_scanner` example
✅ Ensure **SEL pin** is grounded for I2C mode

### Inconsistent Readings

✅ Allow **24-48 hours** for sensor stabilization
✅ Ensure **adequate ventilation** around the sensor
✅ Avoid **direct exposure** to high-VOC sources

### I2C Errors

✅ Confirm **pull-up resistors** are present (**4.7kΩ** recommended)
✅ Reduce **I2C clock speed** if using long wires

---

## Contributing

Contributions are welcome! Please open an **issue** or submit a **pull request** for any:

- **Bug fixes**
- **Additional features**
- **Documentation improvements**

---

## License

This project is licensed under the **MIT License** - see [LICENSE](LICENSE) for details.

---

## Acknowledgements

- **Sensirion AG** for the **SVM30** sensor
- **Espressif Systems** for the **ESP-IDF framework**

=======
Allow 24-48 hours for sensor stabilization

Ensure adequate ventilation around the sensor

Avoid direct exposure to high-VOC sources

I2C Errors


Confirm pull-up resistors are present (4.7kΩ recommended)

Reduce I2C clock speed if using long wires

Contributing

Contributions are welcome! Please open an issue or submit a pull request for any:

Bug fixes

Additional features

Documentation improvements

License

This project is licensed under the MIT License - see LICENSE for details.

Acknowledgements

Sensirion AG for the SVM30 sensor

Espressif Systems for the ESP-IDF framework

