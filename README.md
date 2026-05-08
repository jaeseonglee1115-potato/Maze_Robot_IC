# Maze_Robot_IC (MazeRunner)

MazeRunner is an RTL-based autonomous maze-solving robot system implemented in **SystemVerilog**. This project focuses on hardware architecture, real-time sensor-driven navigation, and timing-closure-oriented digital design.

## Project Highlights

- **Hardware PID motor control**
  - Designed hardware-level PID controllers to regulate dual-motor PWM speed with high precision.
- **Sensor fusion and autonomous navigation**
  - Integrated **Gyro** and **IR** sensor interfaces (via **SPI**) for real-time environment perception.
  - Built an autonomous navigation **FSM** for wall detection and pathfinding.
- **Communication interfaces**
  - Added serial/control wrappers including **UART/BLE command handling** for external or remote control.
- **Speed estimation and control loop**
  - Sensor interface outputs are used by the PID datapath to calculate and continuously correct motor speed.

## System Structure (High Level)

- **Sensor Interface Block**
  - SPI-connected Gyro and IR acquisition
  - Sensor data conditioning for control logic
- **Control/Navigate Block**
  - Maze navigation FSM
  - Decision logic for steering and movement states
- **Motor Control Block**
  - PID controller datapath
  - PWM generation for dual motors
- **Communication Block**
  - UART/BLE command wrapper for host/remote interaction

## Implementation and Timing Closure

- Completed RTL design and synthesis for the full control datapath.
- Generated and analyzed synthesis/timing reports to resolve critical paths.
- Addressed negative slack by pipelining multiplier-heavy and complex combinational stages.
- Achieved timing closure at **363 MHz (2.75 ns)** target clock.

## Tech Stack

- **Language:** SystemVerilog
- **Domain:** RTL Design, Digital Logic, Sensor-Driven Control Architecture
- **Focus Areas:** Hardware control, sensor fusion, synthesis, timing optimization
