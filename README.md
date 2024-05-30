PAES-HARDWARE-AIRMOUSE
Introduction

The PAES-HARDWARE-AIRMOUSE module is a part of the Patient Assistance and Entertainment System (PAES). This module uses an ESP32 microcontroller to create a Bluetooth air mouse. It reads motion data from an IMU sensor and translates it into mouse movements. Additionally, it uses GPIO buttons to implement left and right mouse clicks.
Features

    Bluetooth Mouse: Connects to devices as a Bluetooth mouse.
    Motion Detection: Uses an IMU sensor to detect and translate motion into mouse movements.
    Button Clicks: Supports left and right mouse clicks using GPIO buttons.
    Custom Sensitivity and Delay: Adjustable sensitivity and delay for motion detection and button clicks.

Technologies Used

    Arduino (ESP32)
    IMU Sensor (MPU6050)
    BleMouse Library for Bluetooth mouse functionality

Setup Instructions
Prerequisites

    Arduino IDE
    ESP32 board setup in Arduino IDE
    BleMouse library for Arduino (Link to BleMouse library)
