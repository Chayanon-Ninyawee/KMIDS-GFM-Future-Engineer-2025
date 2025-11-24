# API Documentation

This section provides a complete overview of the software implemented in our self-driving robot system. This document contains a direct link to detailed documentations for every major hardware interface, processing pipeline, and communication layer between the Raspberry Pi 5 and Raspberry Pi Pico 2. Each directory below links to a dedicated README file describing the functionality, dependencies, and data flow within that module across all source code of the components.

### Table of contents

<!-- toc -->

- [Raspberry Pi 5](#raspberry-pi-5)
  - [src](#src)
  - [external](#external)
- [Raspberry Pi Pico 2](#raspberry-pi-pico-2)
  - [external](#external-1)
  - [src/modules/controllers](#srcmodulescontrollers)
  - [src/modules](#srcmodules)
- [Shared](#shared)

<!-- tocstop -->

## Raspberry Pi 5

The Raspberry Pi 5 serves as the primary processing unit of the system, responsible for high-level perception, computation, and communication. It handles sensor fusion from LiDAR and camera modules, performs navigation logic, and manages data exchange with the Pico 2 controller via I²C.

### src

- [**[CLICK HERE] Link to the API Documentation**](raspberry-pi-5/src/README.md)

### external

- [lccv](raspberry-pi-5/external/lccv)
- [rplidar_sdk](raspberry-pi-5/external/rplidar_sdk)

## Raspberry Pi Pico 2

The Raspberry Pi Pico 2 acts as a low-level embedded controller dedicated to precise actuation and sensor interfacing. It governs real-time control loops, servo and motor operation, and communication with sensors like the BNO085 IMU through I²C and UART protocols.

### external

- [BNO08x_Pico_Library](raspberry-pi-pico-2/external/BNO08x_Pico_Library)
- [pico_sdk](raspberry-pi-pico-2/external/pico-sdk)

### src/modules/controllers

- [bno085](raspberry-pi-pico-2/src/modules/controllers/bno085/README.md)
- [motor](raspberry-pi-pico-2/src/modules/controllers/motor/README.md)
- [servo](raspberry-pi-pico-2/src/modules/controllers/servo/README.md)

### src/modules

- [i2c_slave](raspberry-pi-pico-2/src/modules/i2c_slave/README.md)

## Shared

- [i2c](shared/i2c/README.md)
- [types](shared/types/README.md)
