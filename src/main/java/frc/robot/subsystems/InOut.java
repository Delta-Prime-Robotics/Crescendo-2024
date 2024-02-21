// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.Constants.InOutConstants;
import frc.robot.Constants.NeoMotorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class InOut extends SubsystemBase {
 
  //Output Top & Bottom 
  private final CANSparkMax m_FollowerShooter;
  private final CANSparkMax m_LeaderShooter;
  private SparkPIDController shooterPIDController;
  private final CANSparkMax m_intake;

  private static double kFF = 0.5;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  public static double kSetpoint = 50;
  private static final double kMinOutput = -1;
  private static final double kMaxOutput = 1;
  private static final double kMaxRPM = 4800;
  //IR Beam Break
  public static DigitalInput bbInput = new DigitalInput(Constants.InOutConstants.kBeamBreakDIO);

  /** Creates a new InOut. */
  public InOut() {

    //setting CAN ID's for Shooter Motor Controlers
    m_FollowerShooter = new CANSparkMax(InOutConstants.kBottomOutputCanId, MotorType.kBrushless);//Bottom
    m_LeaderShooter = new CANSparkMax(InOutConstants.kTopOutputCanId, MotorType.kBrushless);//Top
    m_FollowerShooter.follow(m_LeaderShooter);

    //Set Current Limit
    m_FollowerShooter.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_LeaderShooter.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
 
    //seting idle to coast
    m_FollowerShooter.setIdleMode(InOutConstants.kShooterIdleMode);
    m_LeaderShooter.setIdleMode(InOutConstants.kShooterIdleMode);

    //Seting Shooter PID Controler
    this.shooterPIDController = m_LeaderShooter.getPIDController();
    this.shooterPIDController.setFF(kFF);
    this.shooterPIDController.setP(kP);
    this.shooterPIDController.setI(kI);
    this.shooterPIDController.setD(kD);
    this.shooterPIDController.setOutputRange(kMinOutput, kMaxOutput);
    
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("setpoint", kSetpoint);
    //setting CAN ID's for Intake Motor Controler
    m_intake = new CANSparkMax(InOutConstants.kIntakeCanId, MotorType.kBrushless);
    m_intake.setIdleMode(InOutConstants.kIntakeIdleMode);
    m_intake.setSmartCurrentLimit(NeoMotorConstants.kNeo550SetCurrent);
    
  }

  //Note detector
  public BooleanSupplier isNoteInIntake() {
    //when the BeamBreak is false there is a note in the Intake
    if (bbInput.get() == false) {
      return () -> true; 
    }
    else{
    return () -> false;
    }
  }

  //Beam Break will return true when there is no Note
  public BooleanSupplier isNoteOutOfIntake() {
    return () -> bbInput.get();
  }

   /**
   * sets the Velocity setpoint of the PIDControler to inputed speed
   * @param speed in rpm
   */
  public void setShooterRef(double speed) {
    shooterPIDController.setReference(speed, ControlType.kVelocity);
  }

  //TO-DO
  //GO THROUGH THIS one by one to test if it works
  public Command shootIntoSpeaker(){
    double shooterVelocity = m_LeaderShooter.getEncoder().getVelocity();
    final double kspeed = 300; //Speed is in RPMs
    
    SequentialCommandGroup sequence = new SequentialCommandGroup();
      sequence.addCommands(
        new InstantCommand(() -> setShooterRef(kspeed)) 
        //if this doesnt work maybe set to RunCommand, it might need to be updating multible times
        .deadlineWith(new WaitUntilCommand(() -> shooterVelocity == kspeed)),
        //This might not work because the lamda is pointing to a boolean and is not a boolen sublyer
        intoShooter(),
        new InstantCommand(() -> setShooterRef(0))
      );
    return sequence;
  }
  
  public Command intoShooter(){
    return new InstantCommand(() -> m_intake.set(0.75))
    .andThen(new WaitCommand(1.5))// or use WaitCommand(IsNotOutOfIntake).withTimeout(1.5)  
    .andThen(new InstantCommand(() -> m_intake.set(0)));
  }

  //if manual Overide is True it will ignore The Beam Break
  public void intakeNote(double speed, boolean maunalOveride){
    if (maunalOveride){
      m_intake.set(speed);
    }
    else if (isNoteInIntake().getAsBoolean()) {
      m_intake.set(0);
    }
    else{
      m_intake.set(speed);
    }
  }

  public InstantCommand intakeStop(){
    return new InstantCommand(() -> m_intake.set(0));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double setpoint = SmartDashboard.getNumber("setpoint",0);
    setpoint = kSetpoint;
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { shooterPIDController.setP(p); kP = p; }
    if((i != kI)) { shooterPIDController.setI(i); kI = i; }
    if((d != kD)) { shooterPIDController.setD(d); kD = d; }
    if((ff != kFF)) { shooterPIDController.setFF(ff); kFF = ff; }
    SmartDashboard.putNumber("shooter volocity", m_LeaderShooter.getEncoder().getVelocity());
  }
  
}
