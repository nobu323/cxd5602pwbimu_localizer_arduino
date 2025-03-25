# Spresense IMU Self-Localization

English | [日本語](./README_jp.md)

This repository contains an Arduino IDE project for the Spresense board that uses the onboard IMU (Inertial Measurement Unit) to perform sensor-based self-localization. The project demonstrates sensor fusion techniques, including calibration, filtering, and orientation estimation, to estimate acceleration, velocity, and position.

## Overview

The code reads raw IMU data (acceleration, rotation speed, and temperature) from the Spresense sensor and processes it through several steps:

- **Calibration & Initialization:** The IMU sensor data is averaged over a predefined measurement period to determine the initial bias and orientation.
- **Filtering:** A causal Gaussian filter is applied to smooth the sensor readings.
- **Orientation Estimation:** The program uses quaternion representations and the Madgwick filter method to fuse accelerometer and gyroscope data. The quaternion update is computed using the Runge–Kutta (RK4) method.
- **Zero Velocity Correction:** A zero-velocity update is implemented to reset drift when the sensor readings indicate near-zero movement.
- **Position Estimation:** Velocity and position are updated using a simple Euler integration based on the corrected acceleration values.

## Features

- **IMU Data Acquisition:** Reads acceleration, gyroscope, and temperature data from the Spresense IMU.
- **Sensor Fusion:** Combines accelerometer and gyroscope data with calibration to provide reliable estimates of orientation.
- **Quaternion-Based Orientation:** Implements quaternion differential equations and RK4 integration for smooth orientation updates.
- **Gaussian Filtering:** Uses a causal Gaussian filter to reduce high-frequency noise in sensor data.
- **Zero-Velocity Correction:** Applies a bias correction when the device is detected to be stationary.
- **Real-Time Data Output:** Logs key computed values (timestamp, sensor data, estimated acceleration, orientation, velocity, and position) via the Serial interface.

## Hardware Requirements

- **Sony Spresense Board:** Ensure your board includes the onboard IMU.
- **IMU Sensor:** The project uses the built-in CXD5602PWBIMU sensor.

## Software Requirements

- **Arduino IDE:** Compatible with the Spresense Arduino libraries.
- **Spresense SDK:** Required to compile and flash the firmware onto the board.

## Getting Started

### Installation

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/yourusername/spresense-imu-self-localization.git
   cd spresense-imu-self-localization
   ```

2. **Open in Arduino IDE:**

   Open the project in the Arduino IDE. Make sure the Spresense board and related libraries are installed.

3. **Compile & Upload:**

   Build and upload the code to your Spresense board. The program automatically starts reading from the IMU once the board is running.

### Running the Application

- Once uploaded, open the Serial Monitor (set at 115200 baud) to view the output.
- The code prints formatted sensor data at regular intervals, including the current orientation (as a quaternion), velocity, and position estimates.

## Code Structure and Algorithm Details

### Initialization and Calibration

- **Initialization Function (`imu_data_initialize`):**  
  Averages a number of samples (defined by `MESUREMENT_FREQUENCY`) to calibrate the accelerometer and gyroscope readings. It also computes the initial quaternion based on gravity and earth rotation.

### Data Processing

- **Gaussian Filtering:**  
  A causal Gaussian filter smooths both the acceleration and rotation speed data using a precomputed kernel.
  
- **Orientation Update:**  
  The quaternion is updated using the RK4 integration method based on the gyroscope readings. This approach ensures a smooth integration of rotational motion.

- **Madgwick Filter:**  
  The code applies the Madgwick algorithm (with custom weights) to fuse accelerometer and gyroscope data, correcting the orientation by comparing the expected and measured gravity direction.

- **Zero-Velocity Correction:**  
  If the computed velocity falls below a dynamic bias threshold, the algorithm resets the velocity to zero to mitigate drift.

### Integration for Velocity and Position

- **Velocity & Position Updates:**  
  After compensating for gravity, the acceleration is integrated over time (using simple Euler integration) to update the velocity and then the position.

## Contributing

Contributions are welcome! Please fork the repository and submit your pull requests. For major changes, please open an issue first to discuss what you would like to change.

## License

Distributed under the MIT License. See `LICENSE` for more information.

---

This README provides an overview of the project and explains the main processing steps implemented in the code. For further details or questions, please feel free to open an issue or contact the repository maintainer.
