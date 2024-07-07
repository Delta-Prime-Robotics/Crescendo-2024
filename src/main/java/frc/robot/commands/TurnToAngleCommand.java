// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCommand extends PIDCommand {
  /** Creates a new TurnToAngleCommand. Will */
  public TurnToAngleCommand(double targetAngleRadians, DriveSubsystem drive) {
    super(
        new PIDController(0.5, 0, 0), //tune please
        // Close loop on heading
        () -> drive.getPose().getRotation().getRadians(),
        // Set reference to target
        targetAngleRadians,
        // Pipe output to turn robot
        output -> drive.drive(0, 0, output, true, true),
        // Require the drive
        drive);
    SmartDashboard.putNumber("setpoint", getController().getSetpoint());
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-1 * Math.PI, Math.PI);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(0.0349066, 0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
