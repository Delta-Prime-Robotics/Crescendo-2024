// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.SpeakerRotateUtil;

public class CalculateAndTurnCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final SpeakerRotateUtil m_rotateUtil;

  public CalculateAndTurnCommand(DriveSubsystem driveSubsystem, SpeakerRotateUtil rotateUtil) {
      this.m_driveSubsystem = driveSubsystem;
      this.m_rotateUtil = rotateUtil;
      addRequirements(driveSubsystem);  // Ensure that only this command uses the subsystem
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DoubleSupplier robotXSupplier = () -> m_driveSubsystem.getPose().getX();
    DoubleSupplier robotYSupplier = () -> m_driveSubsystem.getPose().getY();
    double targetAngle = m_rotateUtil.returnSpeakerAngle(robotXSupplier, robotYSupplier);
        
    // // Start turning to the calculated angle
    // m_driveSubsystem.turnToAngleCommand(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // DoubleSupplier robotXSupplier = () -> m_driveSubsystem.getPose().getX();
      // DoubleSupplier robotYSupplier = () -> m_driveSubsystem.getPose().getY();
      // DoubleSupplier angle = () -> m_rotateUtil.returnSpeakerAngle(robotXSupplier, robotYSupplier);
      // m_driveSubsystem.turnToAngleCommand(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
