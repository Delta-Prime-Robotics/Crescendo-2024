// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
  private static SparkAbsoluteEncoder m_AbsoluteEncoder;
  private static SparkPIDController m_pidController;
  //THESE NEED TO BE VERY LOW  LIKE 0.000001
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private double kMaxOutput = 0.75; 
  private double kMinOutput = -0.75;
  //private double maxRPM = 5700;
  private double setPoint = 0; // in rotations 

  private double kMinRotation = 0.96; //yes this is correct :P arnt AbsoluteEncoders fun
  private double kMaxRotation = 0.25;
 
  
   //public double kIz = 0;
  //public double maxVel, minVel, maxAcc, allowedErr;


  
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
    
    //m_leader.setClosedLoopRampRate(1.5);

    // //Encoder
    m_AbsoluteEncoder = this.m_leader.getAbsoluteEncoder(Type.kDutyCycle);
    
    // Smart Motion Coefficients
    // maxVel = 2000; // rpm
    // maxAcc = 1000;
    // allowedErr = 0.002;
    // //maxAcc = 1500;
    
    // ArmPID and FeedForward
    m_pidController = this.m_leader.getPIDController();
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
    m_pidController.setFeedbackDevice(m_AbsoluteEncoder);
    //m_pidController.setIZone(kIz);
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    //SmartDashboard.putNumber("I Zone", kIz);

    // int smartMotionSlot = 0;
    // m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    // //m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    // m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    // m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    
    
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

  public static Supplier<ArmState> armStateLogic = () -> {
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
    m_pidController.setReference(rotations, ControlType.kPosition);
  }

  //need victors help with this, problem is speed < 0 can't be used.
  public void limitSetRef(double rotations){
    if ((armRotation() != kMaxRotation) || (armRotation() != kMinRotation) && rotations != armRotation()) {
      m_pidController.setReference(rotations, ControlType.kPosition);
    }
    else{
      m_pidController.setReference(armRotation(), ControlType.kPosition);
    }
  }
  
  /**Postion of the Arm in rotations. 
   * 0 to 1, wraps back to zero*/
  public static double armRotation() {
    return m_AbsoluteEncoder.getPosition();
  }

  public double armSpeed() {
    return m_leader.get();
    
  }
  
  public void armRun(double speed){
    //this stops the armRotation untill you try to move backwards
    if ( (armRotation() < kMaxRotation) || (armRotation() > kMinRotation) || speed < 0) {
      m_leader.set(speed);
    }
    else{
      m_leader.set(0);
    }
  // m_leader.set(speed);
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
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    //double iz = SmartDashboard.getNumber("I Zone", 0);
    //  double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    //  double minV = SmartDashboard.getNumber("Min Velocity", 0);
    //  double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    //  double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    //if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    // if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    // if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    // if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    // if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("AbsoluteEncoder",  armRotation());
    SmartDashboard.putString("armState", armStateLogic.get().toString());
    // This method will be called once per scheduler run
  }
}
