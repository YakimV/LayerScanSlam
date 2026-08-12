# LayerScan SLAM

A navigation project for a robot running on an ESP32 and a PC. The robot collects data from the LiDAR and IMU and transmits it via UDP to the PC, where a map of the room is constructed and obstacles are detected.

## Project Structure

* **`firmware_for_robot/`** — firmware for the ESP32 (LiDAR, MPU6050, motors, UDP).
* **`pc/`** — Python software for data processing, SLAM, and the UI.

