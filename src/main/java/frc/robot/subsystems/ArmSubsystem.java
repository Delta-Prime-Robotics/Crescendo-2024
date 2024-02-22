// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.units.*;
import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    m_follower.follow(m_leader, true);
    //set smartCurrentLimits
    m_leader.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_follower.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    //Set IdleMode's
    m_leader.setIdleMode(ArmConstants.kArmIdleMode);
    m_follower.setIdleMode(ArmConstants.kArmIdleMode);
    
    
    // //Encoder
    m_AbsoluteEncoder = this.m_leader.getAbsoluteEncoder(Type.kDutyCycle);

    // ArmPID and FeedForward
    m_pidControler = this.m_leader.getPIDController();
    m_pidControler.setP(0);
    m_pidControler.setI(0);
    m_pidControler.setD(0);
    m_pidControler.setFF(0);
    m_pidControler.setFeedbackDevice(m_AbsoluteEncoder);
    
  }

  public void setRef(double rotations){
    m_pidControler.setReference(rotations, ControlType.kPosition);
  }

  public static enum ArmState {
    AMP,
    SPEAKER,
    GROUND,
    ERECT,
    NOSTATE
  }

  public final ArmState armStateLogic() {
    double tolerance = 0.005;

    if(MathUtil.isNear(ArmConstants.kAmpPosition, armRotation(),tolerance))
    {
      return ArmState.AMP;
    }
    else if(MathUtil.isNear(ArmConstants.kSpeakerPosition, armRotation(),tolerance))
    {
      return ArmState.SPEAKER;
    }
    else if(MathUtil.isNear(ArmConstants.kGroundPosition, armRotation(),tolerance))
    {
      return ArmState.GROUND;
    }

    else if(MathUtil.isNear(ArmConstants.kErectPosition, armRotation(),tolerance))
    {
      return ArmState.ERECT;
    }
    return ArmState.NOSTATE;
  }
  
  /**Postion of the Arm in rotations. 
   * 0 to 1, wraps back to zero*/
  public double armRotation() {
    return this.m_AbsoluteEncoder.getPosition();
  }
  
  public void armRun(double speed){
   m_leader.set(speed);
   SmartDashboard.putNumber("speed", speed);
  }
  
  
  public Command goToAmp() {
    return this.run(()-> setRef(ArmConstants.kAmpPosition));
  }

  public Command goToSpeaker() {
    return this.run(()-> setRef(ArmConstants.kSpeakerPosition));
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("AbsoluteEncoder",  armRotation());
    // This method will be called once per scheduler run
  
    
  }
}
