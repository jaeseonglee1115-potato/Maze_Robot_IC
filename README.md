MazeRunner: Autonomous Maze-Solving Robot System
Overview
MazeRunner is a complete RTL hardware design for an autonomous maze-navigating robot. Implemented in SystemVerilog, this system integrates sensor fusion (Gyroscope and IR), hardware-level PID motor control, and a dedicated finite state machine (FSM) for autonomous pathfinding. The design supports both a remote-controlled command mode via Bluetooth (BLE) and a fully autonomous maze-solving mode, achieving a highly optimized synthesis target of 363 MHz.

Key Features
Autonomous Navigation Logic: FSM-based maze solver capable of dynamic wall detection, pathfinding (Left/Right wall affinity), and dead-end U-turns.

Sensor Fusion Datapath: Real-time environmental tracking utilizing SPI interfaces for an iNEMO Gyroscope (heading/yaw rate) and an onboard ADC (left, right, and center IR sensors).

Hardware PID Motor Control: Precision dual-motor actuation using a custom proportional-integral-derivative (PID) controller to adjust PWM duty cycles based on heading error and IR course-correction data.

BLE Command Processor: Remote control support via UART, processing 16-bit movement and calibration commands.

High-Speed Synthesis: Datapath multipliers and combinational chains are heavily pipelined to meet strict timing constraints, successfully achieving a target clock period of 2.75 ns (363 MHz) using an LVT logic library.

Goal Detection: Hall-effect sensor integration to detect the magnetic goal line, triggering a piezo buzzer fanfare upon completion.

System Architecture
The top-level module (MazeRunner.sv) integrates several key sub-components:

Control & Navigation
cmd_proc: Processes 16-bit commands received from the BLE module (calibrate, set heading, move forward, solve maze).

maze_solve: Takes over datapath control when in solving mode. Uses real-time sensor feedback (lft_opn, rght_opn) to dictate turns and forward movement.

Maps: The movement execution block. Monitors progress and coordinates handshakes (mv_cmplt) with the active controller.

PID: Calculates corrective left/right motor speeds based on desired heading vs. actual heading.

IR_math: Adjusts the desired heading dynamically using differential readings from the IR sensors to keep the robot centered in hallways.

Peripherals & Interfaces
UART_wrapper: Handles RX/TX communication with the external Bluetooth module.

inert_intf: Manages the SPI communication protocol with the iNEMO Gyroscope to track actual heading.

sensor_intf: Interfaces with the ADC over SPI to fetch IR proximity readings and battery voltage.

MtrDrv: Translates desired motor speeds from the PID into drive-strength-scaled PWM signals (lftPWM1/2, rghtPWM1/2).

Repository Structure (Core RTL)
Plaintext
├── MazeRunner.sv            # Top-level integration module
├── cmd_proc.sv              # BLE command processor and state machine
├── maze_solve.sv            # Autonomous maze-solving FSM
├── PID.sv                   # Hardware PID controller for motor speeds
├── navigate.sv              # Movement tracking and execution logic
├── IR_math.sv               # Course correction math using IR diffs
├── MtrDrv.sv                # PWM generation and motor drive scaling
├── inert_intf.sv            # SPI driver for the Gyroscope
├── sensor_intf.sv           # SPI driver for the ADC (IR/Battery)
├── UART_wrapper.sv          # UART Rx/Tx for BLE commands
└── piezo_drv.sv             # Fanfare audio driver for goal detection
Simulation and Verification
The system is rigorously verified using a physics-based simulation environment that models real-world robot drift, acceleration, and sensor lag.

Testbenches
MazeRunner_IDLE_tb.sv: Verifies safe power-up behavior. Ensures PWMs default to midrail, sensors initialize correctly, and the robot remains stationary without explicit commands.

MazeRunner_move_tb.sv: Validates the cmd_proc and PID datapath. Tests calibration, fixed-degree turns, and forward movement while monitoring simulated wheel velocities (omega) and heading alignment.

MazeRunner_SolveMaze_tb.sv: System-level test for the autonomous FSM. Places the physical robot model in a virtual grid to verify successful Left-Wall/Right-Wall navigation, turn logic, and goal-line hall-sensor detection.

External Models Used in Testbenches
RunnerPhysics.sv: Simulates real robot kinematics, including yaw rate (heading_v), wheel angular velocities, and PWM duty cycle responses.

RemoteComm.sv: Simulates the BLE module sending 16-bit operation codes.
