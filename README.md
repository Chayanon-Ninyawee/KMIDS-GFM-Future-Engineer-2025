# Hello! We are **Team KMIDS-GFM**.


<p align="center">
  pic
</p>



# WRO - Future Engineers - Robotics Project Documentation


## Team Members
- **Chayanon Ninyawee (Garfield)** 
- **Supakorn Sunthonthammarat (Pleum)** 
- **Thammatouch Chantasarn (Kao)** 

We are a team of dedicated students with a passion for robotics and innovation. This repository documents our full engineering process, including design, components used, development, testing, and coding of our robot.

<!-- NOTE: Replace all placeholder sections below with your team's actual content and details. -->

##  Table of Contents
- [1. About the Project](#1-about-the-project)
- [2. Mobility Management](#2-mobility-management)
  - [Drivetrain](#21-drivetrain)
    - [Motor](#motor)
  - [Steering](#22-steering)
    - [Servo](#servo)
  - [Chassis Design](#23-chassis-design)
  - [Mounting and Structure](#24-mounting-and-structure)
- [3. Power and Sense Management](#3-power-and-sense-management)
  - [Power Source](#31-power-source)
  - [Sensor and Camera](#32-sensor-and-camera)
    - [LIDAR Sensor](#lidar-sensor)
    - [Fish Eye Lens Camera](#fish-eye-lens-camera)
  - [Processing-Unit](#33-processing-units)
    - [Single-Board Computer](#single-board-computer)
    - [Microcontroller](#microcontroller)
  - [Circuit Diagram](#34-circuit-diagram)
- [4. Obstacle Management](#4-obstacle-management)
- [5. Images](#5-images)
- [6. Performance Videos](#6-performance-videos)
- [7. Source Code](#7-source-code)
- [8. Build Instructions](#8-build-instructions)
- [9. GitHub Usage and Commit History](#9-github-usage-and-commit-history)

---

## 1. About the Project


This project focuses on designing, building, and programming an autonomous robot capable of completing a series of complex obstacle challenges as part of the WRO Future Engineers competition. 

Team KMIDS-GFM was inspired by the challenge of applying engineering principles and problem-solving skills to creatively and efficiently solve complex problems. Pushing forward with our passion for innovation and hands-on learning.

Our goal is to design and build a reliable and efficient system that demonstrates our technical and collaborative skills while serving as a learning experience. We followed a systematic process, including brainstorming, researching, prototyping, testing, and iterating. We maintained detailed documentation for ease of knowledge sharing and a smoother workflow throughout the project.

Our robot is engineered using a custom modular chassis in a rear-wheel drive configuration, controlled using a Raspberry Pi 5 and a Raspberry Pi Pico 2. It utilises a combination of an LIDAR sensor and a fish-eye lens camera to provide an advanced system for obstacle detection and navigation.

Our objective is to create an intelligent robot that is capable of navigating through obstacles with pinpoint precision and speed. 


<!-- TODO: You should add a few paragraphs here about your inspiration, high-level goals, team structure, and overall approach. -->

---
## 2. Mobility Management

<!-- Discussion on design principle -->

<!-- Our robot is designed with precision as our priority, as it is essential for navigating obstacles in the competition arena. -->

- **Drive System:** 2-wheel differential drive (rear wheels).
- **Steering:** Front-wheel steering using S0004m servo.

### 2.1 Drivetrain


### Motor

Motor: N20 Motor 
<!-- link here -->
<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/motor.webp" alt="INSERT MOTOR" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Voltage: 6V </li>
        <li>No-load Speed: 500RPM </li>
        <li>Stall Torque 0.15kg-cm</li>
        <li>Current: 0.023A</li>
        <li>Gear Ratio: 1:30</li>
      </ul>
    </td>
  </tr>
</table>

**Reason for Selection:**
- It is compact and lightweight, allowing us to fit into our robot easily.
- Provides moderate torque that is more than enough for the flat arena.
- Easy to use on the robot and integrate with other parts.

**Mounting:**
- Installed using 3D-printed motor clamps screwed to a detachable motor plate.
  <img src="g" alt="show motor plate" >
- Wires connected to Raspberry Pi Pico 2.
- Rubber wheels are screwed onto the motor shaft.

**Considerations:**
To reach a faster speed, we can upgrade the N20 motor to the N30, which is the easiest upgrade, providing an overall improvement in speed and torque. However, the N30 motor is typically larger and such, might need extra modifications to the robot's chassis and motor clamp to fit properly. It also generally operates at higher voltages and currents, so the power supply needs to be modified to support this upgrade.

### 2.2 Steering


We considered many steering systems, but following our design principle of precision, we decided to implement Ackermann steering geometry to better replicate the precise turning behavior of real-world vehicles. Unlike simpler systems, Ackermann steering has the advantage of smoother turns by moving each wheel at different angles in a turn, reducing the slippage of the tires and improving turn accuracy.

The fundamental principle of Ackermann geometry involves positioning the steering linkage so that a line drawn through both front wheels intersects the rear axle of the robot.

<img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/ackermann%20steering.png">

While this steering geometry is complex to implement, we believe that the advantages it provides are important, especially in obstacle navigation and parking, where precise control and minimized turning radius are essential. It enables smoother manoeuvres and accurate alignment in narrower spaces.

Our implementation involves designing a custom 3D-printed Ackermann steering mechanism. Using CAD to design the mechanism gives us the flexibility to experiment with pivot points and steering angles. Although true Ackermann geometry is difficult to implement at our robot's scale, we tried to approximate the behaviour iteratively by adjusting the servo horns and angles in CAD and prototyping by making smaller changes if it doesn't suit our desired behaviour.

<img src="" alt="GIF of steering" >

### S0004m Servo 
<!-- link here -->
<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/servo.png" alt="INSERT SERVO" >
    </td>
    <td> 
      <h3>Specifications:</h3>
      <ul>
          <li>Rated Torque: 0.6kgf-cm</li>
          <li>Speed: 0.09sec/60°</li>
          <li>Voltage: 5V</li>
          <li>Gearing: Plastic</li>
          <li>Type: Digital </li>
      </ul>
    </td>
  </tr>
</table>

**Reason for Selection:**
- The small size and PWM interface make it easy to control via Raspberry Pi Pico 2.
- It has sufficient torque to steer the front wheels responsively.
- It has balanced speed and stability during turns and lane changes.
- This servo is widely used in hobby robotics, and as such, there is much available documentation and mounting kits.

**Mounting:**
- Screwed directly into a platform plate in front of the chassis.
  <img src="" alt="Servo screwed location" >
  <img src="" alt="close up on the plate and screws" >
- A custom servo horn is attached to the steering linkage.
  <img src="" alt="servo horn" >
  ### Our custom-printed servo horn.
  
- It is positioned at the front, positioned to allow for Ackermann steering geometry.

**Considerations** 
We believe that while the servo used is adequate for the task, it can still be replaced with something more precise. We plan to upgrade to a high-resolution digital servo with a narrower deadband and metal gears for more accurate movement. By also integrating a PWM driver such as the PCA9685, we gain 12-bit resolution control, which gives the robot the ability to make finer adjustments than the Raspberry Pi’s native PWM. 

---

### 2.3 Chassis Design

**Design Overview**

Our chassis was designed with a focus on weight and modularity. The goal is for our chassis to be a stable platform that we can implement the steering geometry onto.

Layout
The layout of the chassis is made to fit the rear-mounted motors and front-mounted steering mechanism. Meanwhile, electronics and sensors are mounted in the centre for ease of wiring. 

  <img src="" alt="Base chassis plate" >

  <img src="" alt="diagram of the position of components" >

Our robot chassis was completely custom-designed and 3D printed using [esun PLA+](https://esun3dstore.com/products/pla-pro), which we found is easy to print with, offering a smoother texture while being lightweight and durable. The chassis was also designed with modularity in mind for additional future components and fixes, with reduced overhangs for printing ease. Apart from the main chassis, the drivetrain and steering modules are mounted on our 3D-printed detachable plates that can be fine-tuned during testing, other components, such as motor clamps, sensor brackets, are designed as independent printable components.

Dimensions: 

Software Used: FreeCAD




### 2.4 Mounting and Structure

- Uses a differential geartrain
- The servo is installed at the front centre of the chassis using our **custom servo mounts**.
  
- A **custom printed servo horn** is attached to a pivoting front wheel or steering linkage to control direction.
- The servo is securely screwed into a platform plate.

---

## 3. Power and Sense Management

### 3.1 Power Source

### [18650 Lithium Ion Battery](https://th.cytron.io/p-3.7v-2000mah-li-ion-battery]()

<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/battery.png" alt="INSERT Battery" width = 100% >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
          <li>Capacity: 2000 mAh</li>
          <li>Normal voltage: 3.7V</li>
          <li>Standard discharge: 10A </li>
          <li>Weight: 48g</li>
      </ul>
    </td>
  </tr>
</table>

The power and sensor systems are crucial to the vehicle's ability to navigate the challenges of the competition. For this project, the vehicle is powered by a 2x 18650 Lithium-Ion battery pack, connected in series to provide a nominal voltage of 7.4V and a combined capacity of around 4000 mAh, depending on the cells used. This setup is capable of delivering a continuous current of around 20 Amps, which is sufficient to supply to the robot for various tasks.

The onboard processing unit, the Raspberry Pi 5, serves as the vehicle's brain. Raspberry Pi recommends a 5V 5A power supply. This is well within the limits of what the two batteries can provide.

  <img src="" alt="Battery location" >

### **Extra: Uninterruptible Power Supply**

To ensure stable operation throughout the competition, we implemented an [EP-0136 Raspberry Pi UPS](https://wiki.52pi.com/index.php?title=EP-0136) (Uninterruptible Power Supply) in our vehicle.

It is powered by a 2x18650 Li-ion battery pack and includes a built-in charging circuit, voltage regulation, and power management. It maintains a stable 5V output to the Raspberry Pi 5 even thoughout fluctuations. It has built-in charging and voltage regulation circuits, allowing continuous operation while also recharging the batteries when external power is connected. This setup ensures that the Raspberry Pi won't shut down unexpectedly, allowing uninterrupted data processing and decision making throughout the run.

### 3.2 Sensor and Camera

### LIDAR Sensor

### [RPLIDAR C1](https://www.slamtec.com/en/C1)

<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/LIDAR.jpg" alt="INSERT LIDAR" width = 100% >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Distance Range:
          <ul>
            <li>White: 0.05~12m (70% Reflectivity)</li>
            <li>Black: 0.05~6m (10% Reflectivity)</li>
          </ul>
        </li>
        <li>Angle Resolution: 0.72°</li>
        <li>Sample Frequency: 8~12Hz (10Hz Typical)</li>
        <li>Range Accuracy: 15mm</li>
        <li>Discharge Rate: 30C</li>
        <br/><br/>
      </ul>
    </td>
  </tr>
</table>

**Reason for Selection:**
- **Precise distance measurement** of obstacles within a wide radius
- **360° scanning** makes it ideal for SLAM (Simultaneous Localisation and Mapping).
- **Compact size and lightweightness** allow easy fitting on our robot.
- **Fast sampling rate** allows real-time mapping and obstacle avoidance.

The LIDAR's sensor driver has been custom-coded to better fit our needs. [Code Link Here]

  <img src="" alt="LIDAR Mount" >

### Fish Eye Lens Camera

### [Fish Eye Lens Raspberry Pi 5MP IR Camera](https://th.cytron.io/p-fish-eye-lense-raspberry-pi-5mp-ir-camera?r=1&language=en-gb&gad_campaignid=18809653822)

<table >
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/RaspberryPI-FE-5MPIRCAM.jpg" alt="INSERT Camera" width = 100% >
    </td>
    <td >
      <h3>Specifications:</h3>
      <ul>
        <li>Voltage: 6V</li>
        <li>Resolution: 5MP </li>
        <li>Focal Length: Adjustable</li>
        <li>Image Sensor: OV5647</li>
        <li>Aperture: F2.35</li>
      </ul>
    </td>
  </tr>
</table>

**Reason for Selection:**
- **130° wide field of view** captures a large area for tracking.
- **Infrared compatibility** enables low-light vision.
- **High-resolution (5MP)** provides a clear image for the robot.
- **Compact size** fits well on our robot.


This setup allows for a wide-angle view, enhancing environmental awareness during both the Open Challenge and Obstacle Challenge. The camera identifies course elements such as walls, pillars, colored markers, parking spaces, and lane lines.

- Detect and differentiate wall positions.

- Identify pillar colours and types.

- Recognise parking zones.

- Track path lines and boundaries.
  


### 3.3 Processing Units

### Single-Board Computer

### [Raspberry Pi 5](https://gammaco.com/gammaco/Raspberry_Pi_GB_89RD014.html)

<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/RP5.webp" alt="INSERT RP5" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Quad-core Arm Cortex-A76 CPU @ 2.4GHz</li>
        <li>8GB LPDDR4X RAM</li>
        <li>Dual 4K HDMI output (60fps)</li>
        <li>PCIe 2.0 support via FPC connector</li>
        <li>2x USB 3.0 ports, 2x USB 2.0 ports</li>
      </ul>
    </td>
  </tr>
</table>

### Microcontroller
### [Raspberry Pi Pico 2](https://th.cytron.io/p-raspberry-pi-pico2-board)

<table>
  <tr>
    <td align="center" width="300" >
      <img src="https://github.com/ExpiredOnions/KMIDS-GFM-Future-Engineer-2025/blob/feature/add-docs/docs/resources/RPP2.webp" alt="INSERT RPP2" >
    </td>
    <td>
      <h3>Specifications:</h3>
      <ul>
        <li>Dual ARM Cortex-M33</li>
        <li>4 MB QSPI flash memory</li>
        <li>520KB SRAM</li>
        <li>2.4GHz 802.11n wireless LAN</li>
        <li>26 multi-function GPIO pins</li>
      </ul>
    </td>
  </tr>
</table>



### 3.4 Circuit Diagram



**Wiring Diagram:**
> *[Include labeled wiring diagram image here]*

  <img src="" alt="Wiring diagram pic" >
  <img src="" alt="Wiring real pic" >
  
**Wiring Summary:**


---

## 4. Obstacle Management

<!-- TODO: Obstacle management discussion should include the strategy for the vehicle to
negotiate the obstacle course for all the challenges. This could include flow
diagrams, pseudo code and source code with detailed comments. -->

The open challenge involves the robot completing three full laps around the field without touching the wall. The size of each side of the field and the direction in which the car drives are randomised.

The obstacle challenge requires the robot to complete three laps whilst avoiding the traffic signs. If the sign is red, then the robot must traverse on the right side and if the pillar is green, the robot must traverse on the left. The direction in which the car drives and the placement of the signs are randomised. After the third lap, the car must find the parking area and park in the area without touching the barriers around it.

### 4.1 Strategy Overview

We divide our strategies into three categories 

**Obstacle Detection and Classification:** The robot identifies the signs and determines their colour and relative position.

**Course Navigation** How to navigate the game field without touching objects, while using the input from the obstacle to steer and pass on the correct side of each sign. 

**Parking:** The robot executes a parallel parking



### 4.2 Flowcharts and Pseudocode
<!-- TODO: Include diagrams or lists that represent logical flow for obstacle avoidance. -->

### 4.3 Source Code Summary
<!-- TODO: Describe key code modules related to obstacle management and explain them briefly. -->

---

## 5. Images

<!-- TODO: Pictures of the team and robot must be provided. The pictures of the robot must
cover all sides of the robot, must be clear, in focus and show aspects of the
mobility, power and sense, and obstacle management. Reference in the
discussion sections 1, 2 and 3 can be made to these pictures. Team photo is
necessary for judges to relate and identify the team during the local and
international competitions. -->


### 5.1 Robot Images
<!-- TODO: Include clear, high-quality photos from top, bottom, front, back, left, and right. -->

<table>
  <tr>
    <td align="center">
      <b>Front View</b><br>
      <img src="" width="300">
    </td>
    <td align="center">
      <b>Back View</b><br>
      <img src="" width="300">
    </td>
    <td align="center">
      <b>Left Side View</b><br>
      <img src="" width="300">
    </td>
  </tr>
  <tr>
    <td align="center">
      <b>Right Side View</b><br>
      <img src="" width="300">
    </td>
    <td align="center">
      <b>Top View</b><br>
      <img src="" width="300">
    </td>
    <td align="center">
      <b>Bottom View</b><br>
      <img src="" width="300">
    </td>
  </tr>
</table>

### 5.2 Internal View
<!-- TODO: Show images of internal layout (wiring, board placement, sensors, motors). -->


---

## 6. Performance Videos

<!-- TODO: The performance videos must demonstrate the performance of the vehicle from
start to finish for each challenge. The videos could include an overlay of
commentary, titles or animations. The video could also include aspects of
section 1, 2 or 3. -->

### 6.1 Challenge 1 - Open Challenge
[Watch on YouTube](#) <!-- TODO: Replace with actual video link -->

### 6.2 Challenge 2 - Obstacle Challenge
[Watch on YouTube](#) <!-- TODO: Replace with actual video link -->

<!-- NOTE: Videos must be at least 30 seconds of continuous autonomous run. You may add overlays and labels. -->

---

## 7. Source Code

### 7.1 Code Structure
<!-- TODO: Explain how your codebase is organized (e.g., folders for different challenges or components). -->

<!-- ### 7.2 Main Code Modules
- Motor Control
- Sensor Input & Processing
- Obstacle Navigation
- Serial Communication (e.g., with camera or sensors)
- Setup and Initialization

### 7.3 Compilation/Upload Instructions
NOTE: List any software/IDE needed (e.g., Arduino IDE), libraries required, and steps to upload to microcontroller. -->

---

## 8. Build Instructions

**Bill of Materials:**

| Component               | Specs                          | Qty | Source/Supplier        |
|------------------------|---------------------------------|-----|------------------------|
| N20 DC Motor  |                                 | 2   |    |
| S0004 Servo            |                                 | 1   |    |
| Raspberry Pi Pico 2    | Microcontroller                 | 1   |    |
| Raspberry Pi 5         | SBC                             | 1   | |
| RPLidar C1             | 360° Lidar                      | 1   | SLAMTEC         |
| 5MP Fish Eye Camera    | Wide-angle, IR-capable          | 1   | Cytron     |
| Chassis Material       | esun pla plus         | 1   | Local / 3D Printed     |
| 3D Printed Mounts      | STL files included              | N/A | User-designed          |
| Wires                  | Assorted lengths/connectors     | —   | Any electronics vendor |

---

### 📁 CAD & Design Files


- ` ` — Base chassis
- ` ` — Motor bracket
- ` ` — Fisheye camera mount
- ` ` — Front steering servo mount

All files will be uploaded under ` ` and ` ` folders. 



### 8.1 3D Printed Parts
<!-- TODO: Mention parts that are 3D printed. Include .STL files in the repo. -->

### 8.2 Tools and Assembly
<!-- TODO: Provide brief assembly instructions or a link to a full manual. -->

---
