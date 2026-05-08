# MazeRunner: Autonomous Maze-Solving Robot System 🤖

## 📌 Overview
**MazeRunner** is a complete RTL hardware design for an autonomous maze-navigating robot. Implemented entirely in **SystemVerilog**, this system integrates real-time sensor fusion (Gyroscope and IR), hardware-level PID motor control, and a dedicated finite state machine (FSM) for autonomous pathfinding. 

The design supports both a remote-controlled command mode via Bluetooth (BLE) and a fully autonomous maze-solving mode. Through critical path analysis and datapath pipelining, the full system was synthesized to achieve a highly optimized target clock frequency of **363 MHz (2.75 ns period)** using an LVT logic library.

---

## 🚀 Key Features

* **Autonomous Navigation FSM:** Dynamic maze solver capable of real-time wall detection, pathfinding (Left/Right wall affinity), and dead-end U-turns.
* **Sensor Fusion Datapath:** Real-time environmental tracking utilizing SPI interfaces for an iNEMO Gyroscope (heading/yaw rate) and an onboard ADC (left, right, and center IR sensors).
* **Hardware PID Motor Control:** Precision dual-motor actuation using a custom proportional-integral-derivative (PID) controller to adjust PWM duty cycles based on heading error.
* **BLE Command Processor:** Remote control support via UART, processing 16-bit movement and calibration commands.
* **Timing Optimization:** Pipelined complex multiplier chains and combinational logic to resolve negative slack and meet aggressive synthesis constraints.

---

## 🏗️ System Architecture

The top-level module (`MazeRunner.sv`) orchestrates several specialized hardware sub-systems:

1. **Control & Navigation**
   * `cmd_proc`: Decodes 16-bit BLE commands (calibrate, set heading, move forward, solve maze).
   * `maze_solve`: Autonomous FSM that takes over datapath control using real-time IR sensor feedback (`lft_opn`, `rght_opn`).
   * `Maps`: Tracks movement progress and coordinates handshakes (`mv_cmplt`).
2. **Datapath & Math**
   * `PID`: Calculates corrective left/right motor speeds based on heading variances.
   * `IR_math`: Dynamically adjusts the desired heading using differential IR readings.
3. **Peripherals & Interfaces**
   * `UART_wrapper`: Handles RX/TX communication with the Bluetooth module.
   * `inert_intf` & `sensor_intf`: SPI drivers for Gyroscope and ADC peripherals.
   * `MtrDrv`: Translates PID speeds into drive-strength-scaled PWM signals (`lftPWM1/2`, `rghtPWM1/2`).

---

## 📂 Repository Structure

```text
📦 MazeRunner-System
 ┣ 📜 MazeRunner.sv            # Top-level integration module
 ┣ 📜 cmd_proc.sv              # BLE command processor
 ┣ 📜 maze_solve.sv            # Autonomous maze-solving FSM
 ┣ 📜 PID.sv                   # Hardware PID controller
 ┣ 📜 navigate.sv              # Movement tracking and execution logic
 ┣ 📜 IR_math.sv               # Course correction math using IR diffs
 ┣ 📜 MtrDrv.sv                # PWM generation and motor drive scaling
 ┣ 📜 inert_intf.sv            # SPI driver for the iNEMO Gyroscope
 ┣ 📜 sensor_intf.sv           # SPI driver for the ADC (IR/Battery)
 ┣ 📜 UART_wrapper.sv          # UART Rx/Tx for BLE commands
 ┗ 📜 piezo_drv.sv             # Fanfare audio driver for goal detection
