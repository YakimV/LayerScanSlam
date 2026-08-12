# LayerScan SLAM

A navigation project for a robot running on an ESP32 and a PC. The robot collects data from the LiDAR and IMU and transmits it via UDP to the PC, where a map of the room is constructed and obstacles are detected.

## Project Structure

* **`firmware_for_robot/`** — firmware for the ESP32 (LiDAR, MPU6050, motors, UDP).
* **`pc/`** — Python software for data processing, SLAM, and the UI.



<img width="1280" height="961" alt="image" src="https://github.com/user-attachments/assets/b6e1a7c0-3815-4fc4-acd5-ff7c74e94222" />



https://github.com/user-attachments/assets/b1253a05-c531-4087-a82d-7b9f25080be2

