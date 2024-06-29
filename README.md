# Quadcopter Drone Flight Controller

## Overview

This repository contains the source code for a quadcopter drone flight controller developed in C++ on an STM32H7 microcontroller. The project leverages various sensors and control algorithms to achieve stable and responsive flight.

## Features

- **Microcontroller**: STM32H7 running at 480 MHz
- **IMU Sensor**: MPU9250 for accelerometer, gyroscope, and magnetometer data
- **Radio Receiver**: Reading PWM signals from 4 channels
- **ESC Control**: Generating 500Hz PWM signals for brushless motors
- **GPS and MTF-01**: For computing (x, y, z) position
- **Attitude Estimation**: Using Extended Kalman Filter, Complementary Filter, and Madgwick Filter
- **Quaternion Calculations**: To avoid gimbal lock and enable efficient spherical rotation interpolation
- **PID Controllers**: For various flight modes including stabilized, acrobatic, and GPS position mode

## AHRS (Attitude and Heading Reference System)

The AHRS fuses data from the accelerometer, gyroscope, and magnetometer using an Extended Kalman Filter, Complementary Filter, or Madgwick Filter to estimate the attitude of the quadcopter.

## PID Control

The project utilizes PID controllers to manage motor power in different flight modes, using the composite design pattern for flexibility and scalability. The composite design pattern allows individual PID controllers to be combined, forming a hierarchy that can manage complex control tasks. Below are the different configurations for various flight modes:

- **Stabilized Mode**:
  - **Simple Attitude Control**:
    - The attitude error is fed into a PID controller, which adjusts the motor power.
  - **Cascaded Control**:
    - The attitude error is processed by a PID controller to produce an angular rate target. This target is then used as the input for another PID controller, which adjusts the motor power based on the angular rate error.

- **Acrobatic Mode**:
  - The angular rate error is processed by a PID controller to directly adjust the motor power.

- **GPS Position Mode**:
  - **Direct Position to Attitude**:
    - The position error in the x, y, and z coordinates is processed by a PID controller to determine the desired attitude. This attitude is then fed into another PID controller to adjust the motor power.
  - **Position to Angular Rate**:
    - The position error is used by a PID controller to determine the desired attitude, which is then processed by another PID controller to produce an angular rate target. This target is then used by yet another PID controller to adjust the motor power based on the angular rate error.
  - **Direct Position to Angular Rate**:
    - The position error is directly processed by a PID controller to produce an angular rate target, which is then used by another PID controller to adjust the motor power based on the angular rate error.