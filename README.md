# Autonomous PPA Robot for BYU ECEN 301 🤖

This repository contains the C++ code for an autonomous robot built for the **ECEN 301** course at Brigham Young University. It uses a **Perception-Planning-Action (PPA)** architecture to track a light source in 3D, avoid obstacles with sonar, and respond to user input.



***

## Key Features
* **3D Light Tracking:** A pan-tilt servo with a four-photodiode array tracks light on both horizontal and vertical axes.
* **Dynamic Obstacle Avoidance:** An ultrasonic sonar sensor detects and maneuvers around objects in real-time.
* **Interactive Speed Control:** A capacitive touch sensor allows for on-the-fly speed adjustments.
* **Power Management:** An onboard circuit monitors battery voltage to ensure stable operation.
* **PPA Architecture:** The code is cleanly structured into Perception, Planning, and Action phases for robust, modular control.

***

## Tech Stack

* **Core Hardware:** Arduino Uno, L298N Motor Driver, Pan-Tilt Servos, 4x Photodiodes, HC-SR04 Sonar, TTP223 Capacitive Sensor, and a battery monitoring circuit.
* **Language & Libraries:** C++ using the standard Arduino framework and `<Servo.h>` library.

***

This project is licensed under the **GPL-3.0** due to its origins in the class repository. See the [LICENSE](LICENSE) file for the full license text.
