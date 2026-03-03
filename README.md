# ASHINA FRC Team 10576 - First Age Swerve Drive Chassis Code

![Java](https://img.shields.io/badge/Java-100%25-orange)
![FRC](https://img.shields.io/badge/FRC-2026%20Season-blue)
![Status](https://img.shields.io/badge/Status-Actively%20Maintained-brightgreen)

## Project Overview

This repository contains the complete robot control code for **Team 10576's** First Age chassis, featuring a state-of-the-art swerve drive system for the 2026 FRC season. The codebase is actively maintained and continuously refreshed to optimize performance and reliability.

## Hardware Architecture

### Drive System
The robot utilizes a **swerve drive** configuration providing omnidirectional movement and precise rotation control:

- **8x Kraken Motors** (X60 or X25 variants)
  - Configured as 2 motors per swerve module
  - Drive motors: Provide forward/backward propulsion for each module
  - Steer motors: Enable independent wheel angle control for each module
  - High torque output ensures rapid acceleration and reliable performance
  - Integrated absolute encoders for precise motor feedback

- **4x SwerveModule Mk5i**
  - Industry-standard swerve module design
  - Integrated drive and steer gearboxes with optimized ratios
  - Compact form factor ideal for tight robot geometries
  - Proven durability in competitive FRC environments

### Navigation & Orientation
- **NavX Gyroscope (IMU)**
  - 9-axis motion tracking for accurate rotation measurement
  - Essential for field-oriented drive and autonomous navigation
  - Provides heading, pitch, and roll data for advanced odometry calculations
  - Enables stable gyro-assisted driving during teleop and autonomous periods

### Drive Control Electronics
- **CTRE SparkMAX or TalonFX Motor Controllers**
  - Communicate via CAN bus for synchronized motor control
  - Provide feedback loop control for consistent performance
  - Support closed-loop velocity and position control modes
  - Enable real-time current limiting for brownout protection

## Software Architecture

### Core Framework: YAGSL (Yet Another Generic Swerve Library)
The codebase is built upon **YAGSL**, a comprehensive swerve drive library that abstracts complex kinematics and motor control:

- **Abstraction Layer**: YAGSL handles raw motor commands and translates them into swerve module movements
- **Kinematics Solver**: Converts robot velocity commands (vx, vy, omega) into individual module velocities and angles
- **Configuration-Driven**: Robot characteristics (wheel base, track width, max velocity) are defined in JSON config files
- **Modular Design**: Easily swap hardware components by updating configuration files without code changes

### Key Components

#### SwerveDrive Subsystem
The central subsystem managing all drive operations:
- Initializes and configures all 4 swerve modules
- Hosts the YAGSL `SwerveDrive` object for kinematics calculations
- Provides high-level commands: `drive()`, `setModuleStates()`, `setHeading()`
- Manages odometry for position tracking throughout the match
- Interfaces with NavX for gyroscope data integration

#### Module Configuration
Each swerve module in the YAGSL config defines:
- **Drive Motor CAN ID**: Controller address for the drive motor
- **Steer Motor CAN ID**: Controller address for the steer motor
- **Steer Encoder Offset**: Calibrated angle offset for wheel zero position
- **Drive Inverted/Steer Inverted**: Motor direction corrections
- **Module Location**: Physical position relative to robot center (x, y coordinates)

#### Odometry & Pose Tracking
- **SwerveDriveOdometry**: Tracks robot position on the field using wheel encoder data
- **Pose2d**: Stores current x, y position and rotation angle
- **Field Simulation**: Test code behavior in simulation without physical robot
- **Autonomous Reset**: Odometry is reset at the start of each autonomous period

#### Gyro Integration
The NavX gyroscope provides:
- Continuous heading measurements for field-oriented control
- Pitch and roll data for detecting ramps or slopes
- Yaw velocity for dynamic rotation feedback
- Zero-drift over time with periodic drift correction

### Motor Control Strategy
1. **Teleop Mode**:
   - Driver input maps to desired velocity vectors (vx, vy, rotation omega)
   - YAGSL kinematics solver calculates required module states
   - Each module's drive motor ramps to target velocity smoothly
   - Steer motors rotate wheels to target angles with PID feedback
   - NavX heading guides field-oriented driving

2. **Autonomous Mode**:
   - Pre-planned trajectories define desired robot motion over time
   - YAGSL converts trajectory goals into module states
   - Closed-loop control ensures actual motion matches planned motion
   - Odometry feedback compensates for minor deviations
   - Gyro heading corrections account for environmental factors

### Performance Optimization
- **CAN Bus Optimization**: Motor controllers communicate at high update rates (100+ Hz)
- **Voltage Compensation**: Motor commands scale with battery voltage for consistent performance
- **Current Limiting**: Prevents brownouts during heavy acceleration
- **Friction Compensation**: Steer motors apply holding torque to maintain wheel angles
- **Velocity Ramping**: Smooth acceleration profiles prevent wheel slip and traction loss

## 2026 Season Configuration

This code is optimized for the 2026 FRC game with:
- Rapid acceleration and deceleration for dynamic gameplay
- Precise strafing movements for positioning
- Reliable autonomous navigation with odometry
- Robust teleop control for intuitive driver experience
- Real-time telemetry for debugging and optimization

## Development & Maintenance

The codebase follows FRC best practices:
- **Modular Subsystems**: Each functional system (Drive, Shooter, Intake) is encapsulated
- **Command-Based Architecture**: WPILib command pattern for clean control flow
- **Configuration Files**: JSON-based hardware configs reduce hardcoded values
- **Testing & Simulation**: WPILib simulation tools validate code before deploying to robot
- **Active Maintenance**: Regular updates and performance tuning throughout the season

## Team Information
- **Team**: ASHINA FRC Team 10576
- **Season**: 2026
- **Chassis Type**: Swerve Drive
- **Primary Language**: Java

---

*This code represents Team 10576's commitment to robotics excellence, combining proven swerve drive technology with active development and optimization practices.*
