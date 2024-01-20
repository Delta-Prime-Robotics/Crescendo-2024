// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {
  
  public final static AHRS m_navx = new AHRS(SPI.Port.kMXP); 
  
  /** Creates a new Sensors. */
  public Sensors() {

    try {
      m_navx.reset();
    }
    catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MSP: " + ex.getMessage(), true);
    }
   
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    
  }
}
