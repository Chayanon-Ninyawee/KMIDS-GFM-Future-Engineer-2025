# API Documentation

This section provides a complete overview of the software implemented in our self-driving robot system. This document contains a direct link to detailed documentations for every major hardware interface, processing pipeline, and communication layer between the Raspberry Pi 5 and Raspberry Pi Pico 2. Each directory below links to a dedicated README file describing the functionality, dependencies, and data flow within that module across all source code of the components.

### Table of contents

<!-- toc -->

- [Raspberry Pi 5](#raspberry-pi-5)
- [Raspberry Pi Pico 2](#raspberry-pi-pico-2)
- [Shared](#shared)

<!-- tocstop -->

## Raspberry Pi 5

The Raspberry Pi 5 serves as the primary processing unit of the system, responsible for high-level perception, computation, and communication. It handles sensor fusion from LiDAR and camera modules, performs navigation logic, and manages data exchange with the Pico 2 controller via I²C.

[**[CLICK HERE] Link to the API Documentation**](raspberry-pi-5/src/README.md)

## Raspberry Pi Pico 2

The Raspberry Pi Pico 2 acts as a low-level embedded controller dedicated to precise actuation and sensor interfacing. It governs real-time control loops, servo and motor operation, and communication with sensors like the BNO085 IMU through I²C and UART protocols.

[**[CLICK HERE] Link to the API Documentation**](raspberry-pi-pico-2/src/README.md)

## Shared

The Shared directory contains all cross-platform components used by both the Raspberry Pi 5 and Raspberry Pi Pico 2. These include common data structures, communication utilities, protocol definitions, and other foundational modules that standardize interactions across the entire system. By centralizing shared logic, this directory ensures consistent data handling, reduces duplication, and maintains interoperability between all subsystems.

[**[CLICK HERE] Link to the API Documentation**](shared/README.md)
