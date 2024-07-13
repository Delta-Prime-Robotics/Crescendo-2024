# Crescendo 2024 
Our robot's name is Wheelie. Wheelie is a swerve drive chassis using the REV Robotics Max Swerve Drive Module. It is the high gear version with a 14T. I used code from Team 2846 FireBears' 2024 Robot and adapted it to enable our robot to use PathPlanner and other advanced swerve drive functionalities. The link to this code can be found [here](https://github.com/firebears-frc/FB2024/blob/bdd0ba19f22cde7e789beca104aab3dc9c107a03/src/main/java/frc/robot/subsystems/Bass.java).

I would also like to credit REV Robotics for their MAXSwerve-Java-Template, which can be found [here](https://github.com/REVrobotics/MAXSwerve-Java-Template).

The robot Wheelie was made for the 2024 season of FIRST called Crescendo. It is a robot of the team Delta Prime Robotics 4473.

## Description

This robot is designed for a drivetrain composed of four MAXSwerve Modules, each configured with:
- Two SPARK MAX motor controllers
- A NEO as the driving motor
- A NEO 550 as the turning motor
- A REV Through Bore Encoder as the absolute turning encoder

To get started, make sure you have calibrated the zero offsets for the absolute encoders in the Hardware Client using the Absolute Encoder tab under the associated turning SPARK MAX devices.

This robot also uses another REV Through Bore Encoder as the absolute turning encoder for the Arm angle. 
Make sure you have calibrated the zero offsets for the absolute encoders in the Hardware Client when the arm is fully resting at a 90 degrese angle to the floor.

This robot also uses KauaiLabs navX2-MXP as the main gyro

### Prerequisites

- SPARK MAX Firmware v1.6.2 - Adds features that are required for swerve

- REVLib v2023.1.2 - Includes APIs for the new firmware features
Configuration

- WPILIB v2024.3.2 - Main Libary for CommandBased Robot and all class features. **THIS IS NOT OPTIONAL**

### Configuration
It is possible that this project will not work for your robot right away Various things like the CAN IDs, PIDF gains, chassis configuration, etc., must be determined for your own robot! These values can be adjusted in the Constants.java file.

