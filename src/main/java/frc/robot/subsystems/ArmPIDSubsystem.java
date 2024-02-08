// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmPIDSubsystem extends PIDSubsystem {
  private final CANSparkMax m_leftArm; 
  private final CANSparkMax m_rightArm;
  private final DutyCycleEncoder m_encoder;

  /** Creates a new ArmPIDSubsystem. */
  public ArmPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0,0));

    //setting CAN ID's for Arm
    m_leftArm = new CANSparkMax(ArmConstants.kArmLeftCanId, MotorType.kBrushless);
    m_rightArm = new CANSparkMax(ArmConstants.kArmRightCanId, MotorType.kBrushless);
    m_rightArm.follow(m_leftArm);
    
    //set smartCurrentLimits
    m_leftArm.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_rightArm.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);

    //Set IdleMode's
    m_leftArm.setIdleMode(ArmConstants.kArmIdleMode);
    m_rightArm.setIdleMode(ArmConstants.kArmIdleMode);
    
    m_encoder = new DutyCycleEncoder(ArmConstants.kArmEncoderDIO);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public void armRun(double speed){
    m_leftArm.set(speed);
   }
}
