# Underwater Robot – STM32F429

An embedded control system developed for an underwater robotic platform as part of the **Robotic Comprehensive Practice** course.

The project combines **STM32F429-based embedded control, FreeRTOS, sensor integration, closed-loop control, serial communication, and autonomous navigation** into a modular underwater robotic system.

**Project period:** 24.06.2024 – 05.08.2024
**Platform:** STM32F429
**Development environment:** Keil µVision / Embedded C

---

## Overview

The goal of this project was to design and implement the electrical control system of an underwater robot, including MCU-based motion and gripper control.

The embedded system integrates multiple sensors, communication interfaces and control modules to support both remote and autonomous operation.

Key components of the system include:

* STM32F429-based embedded control
* FreeRTOS task-based system architecture
* Multi-axis motion control
* Depth and attitude control
* PID-based closed-loop control
* Sensor filtering
* Underwater pressure and IMU sensing
* Serial communication with an external computer / ROS system
* iBUS remote-control communication
* Autonomous control and finite-state-machine logic
* Servo-based gripper control
* Integration with higher-level visual tracking and path-planning modules

---

## System Architecture

```text
                         ┌─────────────────────────┐
                         │       ROS / PC           │
                         │                         │
                         │ Visual Tracking         │
                         │ Path Planning           │
                         └────────────┬────────────┘
                                      │
                               Serial Communication
                                      │
                                      ▼
┌──────────────────────────────────────────────────────────────┐
│                        STM32F429                             │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐  │
│  │                 Application / AUTO                     │  │
│  │                                                        │  │
│  │  State Machine · Autonomous Control · Remote Control  │  │
│  └───────────────────────────┬────────────────────────────┘  │
│                              │                               │
│  ┌───────────────────────────▼────────────────────────────┐  │
│  │                    CONTROL                             │  │
│  │                                                        │  │
│  │  Attitude PID · Depth PID · Velocity Control           │  │
│  │  Sensor Filtering · Servo / Gripper Control             │  │
│  └───────────────┬───────────────────────────────┬────────┘  │
│                  │                               │           │
│  ┌───────────────▼──────────────┐  ┌────────────▼─────────┐ │
│  │          HARDWARE             │  │     COMMUNICATE      │ │
│  │                               │  │                      │ │
│  │ MS5837 · HWT905/JY901         │  │ ROS / PC Serial      │ │
│  │ BMP280 · Other Peripherals    │  │ iBUS Remote Control  │ │
│  └───────────────────────────────┘  └──────────────────────┘ │
│                                                              │
│                         FreeRTOS                              │
└──────────────────────────────────────────────────────────────┘
```

---

## Hardware

### STM32F429

The STM32F429 serves as the main embedded controller and executes the real-time control tasks.

### MS5837

A pressure sensor used for underwater depth measurement.

The measured depth is fed into the depth-control loop for closed-loop depth regulation.

### HWT905 / JY901

An IMU providing attitude-related measurements for orientation and motion control.

### BMP280

Used for internal environmental monitoring, including pressure and temperature measurements.

### Servo System

Servo control is used for actuator functions including the underwater robotic gripper.

---

## Control System

The project implements several closed-loop control components.

### Depth Control

The MS5837 depth measurement is processed and used by a PID controller to regulate the robot's target depth.

```text
Target Depth
     │
     ▼
 ┌─────────┐
 │   PID   │
 └────┬────┘
      │
      ▼
 Thruster / Motor Control
      │
      ▼
 Underwater Robot
      │
      ▼
 MS5837 Depth Sensor
      │
      └─────────────── Feedback
```

### Attitude Control

IMU measurements are used for attitude-related feedback and stabilization.

The control system processes the measured orientation and generates corresponding actuator commands.

### Velocity Control

Velocity-related control and actuator mapping are implemented to translate high-level movement commands into propulsion outputs.

### Sensor Filtering

Sensor data is processed before being used by the control loops to improve the stability and reliability of the feedback signals.

---

## Autonomous Control

The embedded application contains a finite-state-machine-based control structure for switching between different operating states.

The autonomous control layer coordinates:

```text
Sensors
   │
   ▼
State Estimation / Conditions
   │
   ▼
Finite State Machine
   │
   ▼
Control Commands
   │
   ▼
Motion / Actuators
```

This structure allows different behaviors to be separated into explicit control states instead of placing all behavior directly inside the main control loop.

---

## Communication

### ROS / PC Communication

The embedded controller communicates with a higher-level computer through a serial interface.

The communication layer provides a connection between the STM32 control system and higher-level robotic functions such as visual tracking and navigation.

### iBUS Remote Control

An iBUS interface provides remote-control input for manual operation.

This allows the same embedded platform to support both:

* Remote/manual control
* Autonomous control

---

## Navigation and Visual Tracking

The overall robotic system integrates higher-level navigation and perception functions with the embedded controller.

The project involved integration of:

* ROS-based visual tracking
* A* path planning
* D* path planning

These higher-level functions provide navigation-related information to the embedded control system through the communication interface.

The STM32 side is responsible for executing the corresponding low-level motion and actuator control.

---

## Software Architecture

The original project structure is organized according to functional responsibilities:

```text
Underwater-Robot-STM32F429/
│
├── AUTO/
│   ├── auto_control.c / .h
│   ├── auto_need_state.c / .h
│   ├── auto_pid.c / .h
│   └── auto_remote.c / .h
│
├── COMMUNICATE/
│   ├── link232.c / .h
│   └── ibus.c / .h
│
├── CONTROL/
│   ├── attitude_pid/
│   ├── depth_pid/
│   ├── velocity_pid/
│   ├── servo/
│   ├── Sensor_filter/
│   └── ...
│
├── HARDWARE/
│   ├── MS5837/
│   ├── HWT905/
│   ├── BMP280/
│   └── ...
│
├── FreeRTOS/
│
├── CORE/
│   └── startup_stm32f429xx.s
│
├── SYSTEM/
│
├── USER/
│   ├── FreeRTOS.uvprojx
│   ├── main.c / .h
│   ├── stm32f4xx_it.c / .h
│   └── system_stm32f4xx.c / .h
│
├── docs/
│   ├── project-report.pdf
│   ├── project-presentation.pdf
│   └── demo.mp4
│
├── .gitignore
└── README.md
```

### Functional Layers

| Layer          | Responsibility                                              |
| -------------- | ----------------------------------------------------------- |
| `AUTO/`        | Autonomous behavior, state machine and control coordination |
| `COMMUNICATE/` | ROS/PC serial communication and remote-control interfaces   |
| `CONTROL/`     | PID controllers, actuator control and sensor processing     |
| `HARDWARE/`    | Sensor and peripheral drivers                               |
| `FreeRTOS/`    | Real-time operating system                                  |
| `CORE/`        | Cortex-M4 startup and core support                          |
| `SYSTEM/`      | Low-level system functions                                  |
| `USER/`        | Application entry point and project configuration           |

The repository intentionally preserves the original functional organization of the project while documenting the architecture in terms of application, control, hardware and middleware responsibilities.

---

## Development Challenges

### Closed-Loop Control

Underwater operation requires stable control despite changing sensor measurements and external disturbances.

The project therefore required the integration of sensor feedback, PID controllers and actuator commands into a continuous control loop.

### Multi-Sensor Integration

Different sensors provide different types of measurements, including depth, attitude and internal environmental information.

The hardware and filtering layers were designed to process these measurements before they were used by higher-level control functions.

### Real-Time System Integration

The robot combines multiple tasks including sensing, communication, control and actuator operation.

FreeRTOS was used to structure these concurrent real-time activities and separate system functions into manageable tasks.

### Manual and Autonomous Operation

The control system supports both remote-control input and autonomous operation.

The application layer therefore needs to coordinate different control modes while maintaining consistent low-level actuator control.

---

## Demonstration

Project demonstrations, photographs and additional technical material are available in the [`docs/`](./docs/) directory.

If a demonstration video is included, it can be found at:

```text
docs/demo.mp4
```

---

## What I Learned

This project provided practical experience in:

* STM32F429 embedded development
* Embedded C programming
* FreeRTOS-based real-time systems
* PID control
* Sensor integration and filtering
* Depth and attitude control
* Serial communication
* ROS-based robotic system integration
* Autonomous control and state machines
* Path planning
* Hardware/software system integration

The project strengthened my understanding of how **low-level embedded control interacts with higher-level robotic perception and navigation**, from sensor measurements and control loops to communication and autonomous behavior.

---

## Project Context

Developed as part of the **Robotic Comprehensive Practice** course at Chongqing University.

**Project period:** 24.06.2024 – 05.08.2024
**Role:** Embedded control system design and implementation
