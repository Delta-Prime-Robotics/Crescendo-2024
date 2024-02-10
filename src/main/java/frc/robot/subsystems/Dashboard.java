// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Dashboard extends SubsystemBase {
  private static InOut m_InOut;
  private DriveSubsystem m_Drive;
  private ArmSubsystem m_Arm;

  /** Creates a new Shuffleboard. */
  public Dashboard(DriveSubsystem driveSubsystem, InOut inOutSubsystem, ArmSubsystem armSubsystem) {
    m_Drive = driveSubsystem;
    m_InOut = inOutSubsystem;
    m_Arm = armSubsystem;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Get yaw value", Sensors.m_navx.getAngle());
    SmartDashboard.putNumber("Get Heading", m_Drive.getHeading());
    SmartDashboard.putBoolean("Note In Intake", m_InOut.bbInput.get());
    SmartDashboard.putBoolean("Hitting", m_InOut.m_Hitting);
    SmartDashboard.putNumber("Velocity", m_InOut.m_rate);
    SmartDashboard.putNumber("Voltage", m_InOut.m_Volatage);

  }

  protected void execute() {
    
  }
}
