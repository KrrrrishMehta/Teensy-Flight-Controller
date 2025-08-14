# Agastya V1.0 - Quadcopter Flight Controller



Welcome to the repository for the **Agastya V1.0**, a custom-built quadcopter flight controller project by Team Phoenix. This project includes the complete firmware and details about the custom-designed Printed Circuit Board (PCB).

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Hardware](#-hardware)
  - [Custom PCB: Agastya V1.0](#custom-pcb-agastya-v10)
  - [Components](#components)
- [Software & Firmware](#-software--firmware)
  - [Key Features](#key-features)
  - [Control System](#control-system)
- [Getting Started](#-getting-started)
  - [Prerequisites](#prerequisites)
  - [Setup & Flashing](#setup--flashing)
- [Flight & Tuning](#-flight--tuning)
  - [Pre-flight Checklist](#pre-flight-checklist)
  - [PID Tuning](#pid-tuning)
- [Contributing](#-contributing)

---

## 🚁 Overview

This project is a complete flight controller solution for a quadcopter. The firmware is written in C++ for the Arduino environment and is designed to run on a custom PCB, the "Agastya V1.0". The system reads data from an Inertial Measurement Unit (IMU), processes it through a Kalman filter, takes commands from an RC receiver, and uses a PID control loop to stabilize the drone and control its flight.

---

## 🔩 Hardware

### Custom PCB: Agastya V1.0

The heart of this project is a custom-designed PCB that integrates all the necessary components for a clean and reliable build.

[![Agastya V1.0 PCB Layout](https://github.com/user-attachments/assets/21d76989-823c-4ece-abcd-b083f9e7eff6)]
*Assembly diagram for the Agastya V1.0 flight controller board.*
![Image](https://github.com/user-attachments/assets/60e55672-9729-45f9-8bd4-3bc01fb2b5a7)
*PCB Assembly for the Agastya V1.0 flight controller board.*
![Image](https://github.com/user-attachments/assets/4b31935b-1ef0-4f12-ae67-0058d89f76e0)
### Components

- **Microcontroller:** An Arduino-compatible microcontroller (e.g., Teensy, Arduino Nano/Pro Mini).
- **IMU:** MPU-6050 Gyroscope & Accelerometer.
- **RC Receiver:** A receiver that outputs a PPM (Pulse Position Modulation) signal.
- **ESCs:** Four Electronic Speed Controllers for brushless motors.
- **Optional:** A barometer for altitude hold and a display for telemetry (connections are available on the PCB).

---

## 💻 Software & Firmware

The firmware is designed to be robust, efficient, and tunable. It's built on standard Arduino libraries for broad compatibility.

### Key Features

- **Sensor Fusion:** A 1D Kalman Filter is implemented to fuse noisy accelerometer data with drifty gyroscope data, providing a stable angle estimate for roll and pitch.
- **Gyro Calibration:** An automatic gyroscope calibration sequence runs at startup to cancel out sensor bias, ensuring stable flight.
- **PPM Receiver Support:** Reads up to 8 channels from a PPM RC receiver.
- **Fixed-Time Loop:** The main control loop runs at a consistent **250Hz (4ms)**, which is critical for stable PID performance.
- **Safety Features:**
  - **Arming Check:** The drone will not arm until a specific switch is activated on the transmitter.
  - **Throttle Cut-off:** When the throttle stick is at its lowest position, the motors are stopped, and the PID controllers are reset to prevent integral windup.

### Control System

The flight controller uses a cascaded PID control system. The current code is configured for **Rate (Acro) Mode**.

1.  **Rate PID Controller:** This is the inner loop that controls the drone's angular velocity (how fast it rotates). It takes the desired rotation rate from the pilot's sticks and works to match the drone's actual rotation rate (from the gyro) to that target.
2.  **Angle PID Controller (Level Mode):** *This loop is present in the code but is currently commented out.* When enabled, it forms the outer loop. It takes a desired angle from the pilot's sticks and calculates the necessary rotation rate to achieve that angle. This rate is then fed into the Rate PID controller.

---

## 🚀 Getting Started

### Prerequisites

1.  [Arduino IDE](https://www.arduino.cc/en/software)
2.  Required Libraries:
    - `<Wire.h>` (usually included with Arduino IDE)
    - `<Servo.h>` (usually included with Arduino IDE)
    - `<PulsePosition.h>` (Install via the Arduino Library Manager)
3.  Assembled Agastya V1.0 PCB or a breadboard equivalent.

### Setup & Flashing

1.  **Clone the Repository:**
    ```bash
    git clone [your-repository-url]
    ```
2.  **Open in Arduino IDE:** Open the `.ino` file in the Arduino IDE.
3.  **Select Board & Port:** Go to `Tools > Board` and select the microcontroller you are using. Then, select the correct COM port under `Tools > Port`.
4.  **Upload:** Click the "Upload" button to flash the firmware to the board.

---

## 🛠️ Flight & Tuning

**⚠️ WARNING: Always remove propellers before testing, calibrating, or flashing the flight controller.**

### Pre-flight Checklist

1.  Ensure the battery is fully charged.
2.  Place the drone on a perfectly level surface before powering on to ensure correct gyro calibration.
3.  Power on your transmitter first, then the drone.
4.  Check that the arming switch works as expected. The motors should not spin until armed.
5.  Perform a quick control check (without props) to ensure roll, pitch, and yaw respond correctly.

### PID Tuning

PID tuning is critical for stable, responsive flight. The gains are located at the top of the source code file.

```cpp
// --- RATE PID GAINS (Tuning Parameters) ---
float PRateRoll = 0.6;
float PRatePitch = PRateRoll;
float PRateYaw = 0;

float IRateRoll = 0.0;
float IRatePitch = IRateRoll;
float IRateYaw = 0;

float DRateRoll = 0.000;
float DRatePitch = DRateRoll;
float DRateYaw = 0;
```

- **P (Proportional):** This is the main gain. Increase it for a sharper, more responsive feel. If it's too high, the drone will oscillate rapidly.
- **I (Integral):** This gain corrects for drift and external forces like wind. Increase it to help the drone hold its angle. If it's too high, the drone will feel "sluggish" and may oscillate slowly.
- **D (Derivative):** This acts as a dampener. Increase it to smooth out the "kick" from the P-gain and reduce overshoot. If it's too high, the motors may get hot and you'll hear high-frequency noise.

**Tuning Process:**
1.  Start with I and D gains at zero.
2.  Increase the P-gain until you see fast oscillations, then back it off by about 20-30%.
3.  Increase the D-gain to smooth out the flight.
4.  Finally, slowly increase the I-gain to help the drone resist drift.

---

## 🤝 Contributing

Contributions are welcome! If you have improvements for the code or hardware, please feel free to fork this repository and submit a pull request.
