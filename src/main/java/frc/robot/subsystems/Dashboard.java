// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.subsystems.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Dashboard extends SubsystemBase {
  DriveSubsystem m_DriveSubsystem;
  
  
 
  /** Creates a new Shuffleboard. */
  public Dashboard(DriveSubsystem aDrive) {
    m_DriveSubsystem = aDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Get yaw value", Sensors.m_navx.getAngle());
    SmartDashboard.putNumber("Get Heading", m_DriveSubsystem.getHeading());
    SmartDashboard.putBoolean("BB Boolean", InOut.getBB());
  }

  protected void execute() {
    
  }
}
