// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase {

  //Setting arms and Encoder
  private final CANSparkMax m_leader; //left arm
  private final CANSparkMax m_follower; //right arm
  private SparkAbsoluteEncoder m_AbsoluteEncoder;
  private SparkPIDController m_pidControler;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_leader = new CANSparkMax(ArmConstants.kArmLeftCanId, MotorType.kBrushless);
    m_follower = new CANSparkMax(ArmConstants.kArmRightCanId, MotorType.kBrushless);
    //makeing left arm the leader
    m_follower.follow(m_leader);
    //set smartCurrentLimits
    m_leader.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_follower.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    //Set IdleMode's
    m_leader.setIdleMode(ArmConstants.kArmIdleMode);
    m_follower.setIdleMode(ArmConstants.kArmIdleMode);
    m_leader.burnFlash();
    m_follower.burnFlash();
    
    //Encoder
    m_AbsoluteEncoder = this.m_leader.getAbsoluteEncoder(Type.kDutyCycle);
    m_AbsoluteEncoder.setZeroOffset(0);// set this so it equals 0 when arm is touching ground. 
    //ArmPID and FeedForward
    m_pidControler = this.m_leader.getPIDController();
    m_pidControler.setP(0);
    m_pidControler.setI(0);
    m_pidControler.setD(0);
    m_pidControler.setFF(0);
    m_pidControler.setFeedbackDevice(m_AbsoluteEncoder);
    
  }

  public void armRun(double speed){
   m_leader.set(speed);
  }
  
  
  public void goToAmp() {
    double kAmpPosition = 0; //In Rotations
    m_pidControler.setReference(kAmpPosition, ControlType.kPosition); 
  }

  public void goToSpeaker() {
    double kSpeakerPosition = 0; //In Rotations
    m_pidControler.setReference(kSpeakerPosition, ControlType.kPosition);

  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
