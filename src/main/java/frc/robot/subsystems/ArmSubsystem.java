// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase {
  //Setting arms and Encoder
  private final CANSparkMax m_leftArm; 
  private final CANSparkMax m_rightArm;
  private final DutyCycleEncoder m_encoder;
  
  public PIDController m_armController = new PIDController(0,0,0,1);
  public ArmFeedforward m_Feedforward = new ArmFeedforward(0,0,0,0);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_leftArm = new CANSparkMax(ArmConstants.kArmLeftCanId, MotorType.kBrushless);
    m_rightArm = new CANSparkMax(ArmConstants.kArmRightCanId, MotorType.kBrushless);
    //makeing left arm the leader
    m_rightArm.follow(m_leftArm);

    //set smartCurrentLimits
    m_leftArm.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_rightArm.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);

    //Set IdleMode's
    m_leftArm.setIdleMode(ArmConstants.kArmIdleMode);
    m_rightArm.setIdleMode(ArmConstants.kArmIdleMode);

    m_encoder = new DutyCycleEncoder(ArmConstants.kArmEncoderDIO);
    m_armController = new PIDController(0,0,0,1);
    m_Feedforward = new ArmFeedforward(0,0,0,0);
  }

  public DoubleSupplier getPosition() {
    //TO-DO
    //Add maths and stuffs to figure out where the arm is in its arc
    //aka convert Apsolute Position (0 to 1) to Position/angle in Radians(?)
    return () -> m_encoder.get();
  }

  public void armRun(double speed){
   m_leftArm.set(speed);
  }

  public void setArmVoltage(double voltage) {
    //voltage constrants?
    m_leftArm.setVoltage(voltage);
  }

  PIDCommand ampPosition = new PIDCommand(
      m_armController,
      getPosition(), 
      ArmConstants.kAmpSetpoint, 
      output -> setArmVoltage(output + m_Feedforward.calculate(ArmConstants.kAmpSetpoint, ArmConstants.kArmVelocity)), 
      this);
  //make a Speaker command? its so close to the intake that it might not need it
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
