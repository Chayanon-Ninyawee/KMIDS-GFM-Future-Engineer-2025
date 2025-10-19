#  API Documentation

This section provides a complete overview of the software implemented in our self-driving robot system. This document contains a direct link to detailed documentations for every major hardware interface, processing pipeline, and communication layer between the Raspberry Pi 5 and Raspberry Pi Pico 2. Each directory below links to a dedicated README file describing the functionality, dependencies, and data flow within that module across all source code of the components.

### Table of contents

<!-- toc -->

- [Raspberry Pi 5](#raspberry-pi-5)
  * [external](#external)
  * [modules](#modules)
  * [processors](#processors)
  * [types](#types)
  * [utils](#utils)
- [Raspberry Pi Pico 2](#raspberry-pi-pico-2)
  * [external](#external-1)
  * [external](#external-2)
  * [src/modules/controllers](#srcmodulescontrollers)
  * [src/modules](#srcmodules)
- [Shared](#shared)

<!-- tocstop -->

## Raspberry Pi 5

The Raspberry Pi 5 serves as the primary processing unit of the system, responsible for high-level perception, computation, and communication. It handles sensor fusion from LiDAR and camera modules, performs navigation logic, and manages data exchange with the Pico 2 controller via I²C.

### external

- [lccv](code\raspberry-pi-5\external\lccv)
- [relidar_sdk](code\raspberry-pi-5\external\rplidar_sdk)

### modules
- [camera](code/raspberry-pi-5/src/modules/camera/README.md)
- [i2c_master](code/raspberry-pi-5/src/modules/i2c_master/README.md)
- [lidar](code/raspberry-pi-5/src/modules/lidar/README.md)
- [pico2](code/raspberry-pi-5/src/modules/pico2/README.md)

### processors
- [camera](code/raspberry-pi-5/src/processors/camera/README.md)
- [combined](code/raspberry-pi-5/src/processors/combined/README.md)
- [lidar](code/raspberry-pi-5/src/processors/lidar/README.md)

### types
- [types](code/raspberry-pi-5/src/types/README.md)

### utils
- [direction](code/raspberry-pi-5/src/utils/direction/README.md)
- [log_reader](code/raspberry-pi-5/src/utils/log_reader/README.md)
- [logger](code/raspberry-pi-5/src/utils/logger/README.md)
- [pid_controller](code/raspberry-pi-5/src/utils/pid_controller/README.md)
- [ring_buffer](code/raspberry-pi-5/src/utils/ring_buffer/README.md)

## Raspberry Pi Pico 2

The Raspberry Pi Pico 2 acts as a low-level embedded controller dedicated to precise actuation and sensor interfacing. It governs real-time control loops, servo and motor operation, and communication with sensors like the BNO085 IMU through I²C and UART protocols.

### external

- [BNO08x_Pico_Library](code\raspberry-pi-pico-2\external\BNO08x_Pico_Library)
- [pico_sdk](code\raspberry-pi-pico-2\external\pico-sdk)

### src/modules/controllers
- [bno085](raspberry-pi-pico-2/src/modules/controllers/bno085/README.md)
- [motor](raspberry-pi-pico-2/src/modules/controllers/motor/README.md)
- [servo](raspberry-pi-pico-2/src/modules/controllers/servo/README.md)

### src/modules
- [i2c_slave](raspberry-pi-pico-2/src/modules/i2c_slave/README.md)

## Shared

-  [i2c](code\shared\i2c\README.md)
-  [types](code\shared\types\README.md)