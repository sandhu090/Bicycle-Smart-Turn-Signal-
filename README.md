Smart Bicycle Turn Signal & Brake Light System

This project is a smart bicycle lighting system designed to improve rider visibility and safety through automatic and manual signaling. The system supports dual-mode operation, allowing turn signals and brake lights to be triggered automatically based on sensor data, while still providing manual control for the rider.

The system is built around ESP32 microcontrollers programmed in C/C++ using the Arduino IDE. Wireless communication between modules is handled using ESP-NOW, enabling low-latency, peer-to-peer data exchange without requiring a Wi-Fi network. IMU sensors are used to measure handlebar rotation relative to the bike frame in order to determine left and right turns, while Hall-effect sensors detect brake lever engagement to activate the brake light.

The firmware is structured using RTOS-style task separation to independently manage sensor sampling, wireless communication, and LED output control. This design minimizes CPU blocking and ensures fast, reliable, real-time system behavior. The result is a responsive and modular embedded system suitable for prototyping and further expansion.


