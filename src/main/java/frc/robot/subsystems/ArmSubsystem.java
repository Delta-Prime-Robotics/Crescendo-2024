// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase {

  //Setting arms and Encoder
  private final CANSparkMax m_leader; //left arm
  private final CANSparkMax m_follower; //right arm
  private static SparkAbsoluteEncoder m_AbsoluteEncoder;
  private static SparkPIDController m_pidControler;
  private static double kMaxSpeakerAngle = 0.06;
  private static double kMinSpeakerAngle = 0.03;
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

  public static enum ArmState {
    AMP("AMP"),
    SPEAKER("SPEAKER"),
    GROUND("GROUND"),
    ERECT("ERECT"),
    NOSTATE("NOSTATE");

    private final String stringValue;

    ArmState(String stringValue) {
        this.stringValue = stringValue;
    }

    @Override
    public String toString() {
        return stringValue;
    }
  }

  public Supplier<ArmState> armStateLogic = () -> {
    double tolerance = 0.005;
    if(MathUtil.isNear(ArmConstants.kAmpPosition, armRotation(),tolerance)){
      return ArmState.AMP;
    }
    else if(MathUtil.isNear(ArmConstants.kSpeakerPosition, armRotation(),tolerance)){
      return ArmState.SPEAKER;
    }
    else if(MathUtil.isNear(ArmConstants.kGroundPosition, armRotation(),tolerance, 0, 1)){
      return ArmState.GROUND;
    }
    else if(MathUtil.isNear(ArmConstants.kErectPosition, armRotation(),tolerance)){
      return ArmState.ERECT;
    }
    return ArmState.NOSTATE;
  };

  public static void setRef(double rotations){
    m_pidControler.setReference(rotations, ControlType.kPosition);
  }
  
  /**Postion of the Arm in rotations. 
   * 0 to 1, wraps back to zero*/
  public double armRotation() {
    return m_AbsoluteEncoder.getPosition();
  }
  
  public void armRun(double speed){
    //this stops the armRotation untill you try to move backwards
    if ( (armRotation() < 0.25) || (armRotation() > .96) || speed < 0) {
      m_leader.set(speed);
    }
    else{
      m_leader.set(0);
    }
  }
  public Command armToSpeakerCommand(){
  return new RunCommand(()-> getArmInPositionSpeaker(), this)
  .until(()->armAngleInSpeakerRange())
  .finallyDo(() -> m_leader.stopMotor());
  }
  
  public void getArmInPositionSpeaker()
  {
    if (armAngleInSpeakerRange())
    {
        m_leader.set(0);
    }
    else
    {
      double speed = .3;
      if ( this.armRotation() > kMaxSpeakerAngle && this.armRotation() < 0.8)
        speed *= -1.0;

      m_leader.set(speed);
    }
  }

  public Command getArmInGroundPostion() {
    return this.run(()-> armRun(-0.3))
    .until(()->armAngleInGroundRange())
    .finallyDo(()-> armRun(0));
  }


  public boolean armAngleInSpeakerRange()
  {
    // These need to be constant values that we can update
    return ((this.armRotation() > kMinSpeakerAngle) &&
            (this.armRotation() < kMaxSpeakerAngle));
  }

  public boolean armAngleInGroundRange()
  {
    // These need to be constant values that we can update
    return MathUtil.isNear(ArmConstants.kGroundPosition, armRotation(), 0.0005, 0, 1);
  }

  
  
//  public Command goToAmp() {
//  return this.run(()-> setRef(ArmConstants.kAmpPosition));
// }

//  public Command goToSpeaker() {
//    return this.run(()-> setRef(ArmConstants.kSpeakerPosition));
//  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("AbsoluteEncoder",  armRotation());
    SmartDashboard.putString("armState", armStateLogic.get().toString());
    // This method will be called once per scheduler run
  }
}
