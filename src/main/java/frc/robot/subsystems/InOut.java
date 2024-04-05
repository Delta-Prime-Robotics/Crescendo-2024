// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.Constants.InOutConstants;
import frc.robot.Constants.NeoMotorConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class InOut extends SubsystemBase {
 
  //Output Top & Bottom 
  private final CANSparkMax m_FollowerShooter;
  private final CANSparkMax m_LeaderShooter;
  private static RelativeEncoder m_Encoder;
  private SparkPIDController shooterPIDController;
  private static double kFF = 0.0002;
  private static double kP = 0.0001;
  private static double kI = 0;
  private static double kD = 0;
  public static final double kSetpoint = 3800;
  private static final double kMinOutput = -1;
  private static final double kMaxOutput = 1;
  private static final double kMaxRPM = 4800;
  public static DigitalInput m_LimitSwitch = new DigitalInput(0);

  //intake
  private final CANSparkMax m_intake;
  public SparkLimitSwitch m_bbLimitSwitch;

  //IR Beam Break public static DigitalInput bbInput = new DigitalInput(Constants.InOutConstants.kBeamBreakDIO);
  
  //Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  /** Creates a new InOut. */
  public InOut() {
    //setting CAN ID's for Shooter Motor Controlers
    m_LeaderShooter = new CANSparkMax(InOutConstants.kTopOutputCanId, MotorType.kBrushless);//Top
    m_FollowerShooter = new CANSparkMax(InOutConstants.kBottomOutputCanId, MotorType.kBrushless);//Bottom
    m_LeaderShooter.setInverted(true);
    m_FollowerShooter.follow(m_LeaderShooter);
    //Set Current Limit
    m_FollowerShooter.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_LeaderShooter.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    //seting idle to coast
    m_FollowerShooter.setIdleMode(InOutConstants.kShooterIdleMode);
    m_LeaderShooter.setIdleMode(InOutConstants.kShooterIdleMode);
    //Shooter Encoder 
    m_Encoder = this.m_LeaderShooter.getEncoder();

    //Seting Shooter PID Controler
    shooterPIDController = m_LeaderShooter.getPIDController();
    this.shooterPIDController.setFF(kFF);
    this.shooterPIDController.setP(kP);
    this.shooterPIDController.setI(kI);
    this.shooterPIDController.setD(kD);
    this.shooterPIDController.setOutputRange(kMinOutput, kMaxOutput);
    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("setpoint", kSetpoint);
    
    //settings for Intake Motor
    m_intake = new CANSparkMax(InOutConstants.kIntakeCanId, MotorType.kBrushless);
    m_intake.setIdleMode(InOutConstants.kIntakeIdleMode);
    m_intake.setSmartCurrentLimit(NeoMotorConstants.kNeo550SetCurrent);

    m_bbLimitSwitch = m_intake.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_bbLimitSwitch.enableLimitSwitch(false);
   
  }

  public boolean IsNoteInIntake() {
    return !m_LimitSwitch.get();
  }

   /**
   * sets the Velocity setpoint of the PIDControler to inputed speed
   * @param speed in rpm
   */
  public void setShooterRef(double speed) {
    shooterPIDController.setReference(speed, ControlType.kVelocity);
    //NO TOUCHY
  }

  public Command spinUpShooter() {
    return this.run(() -> setShooterRef(kSetpoint))
    .finallyDo(()-> setShooterRef(0));
  }
  
  private static double shooterVelocity() {
    return -m_Encoder.getVelocity(); 
    //NO TOUCHY
  }

  
  public Command shootIntoSpeaker(){
    final double kspeed = kSetpoint; //Speed is in RPMs
    
    SequentialCommandGroup group = new SequentialCommandGroup(
      new ParallelDeadlineGroup(
          new WaitCommand(0.4 ), 
          new InstantCommand(() -> setShooterRef(kspeed))
      ),
      intoShooter(),
      new InstantCommand(() -> setShooterRef(0)));
    return group;
  }
  
  public Command intoShooter() {
    return new InstantCommand(() -> m_intake.set(1))
    .andThen(new WaitCommand(0.5))// or use WaitCommand(IsNotOutOfIntake).withTimeout(1.5)  
    .andThen(new InstantCommand(() -> m_intake.set(0)))
    .andThen(new InstantCommand(()-> setShooterRef(0)));
  }

  public Command intakeCommand(double speed) {
    
    return new RunCommand(()-> setIntakeSpeed(speed), this)
    .finallyDo(()-> m_intake.stopMotor());
    //This stops the motor at end of command or during a interupt
  }

  public Command autoIntakeCommand(double speed) {
    
      return new InstantCommand(()-> setIntakeSpeed(speed), this)
        // Sets Motor speed
        // .unless(()-> IsNoteInIntake()) //this might need 
        //This will only run the command
        //if the note is not in the intake 
      .andThen(
        new WaitUntilCommand(
          ()-> IsNoteInIntake()
        )
      )
        //This will stop the command when the note is in the Intake
      .finallyDo(()-> m_intake.stopMotor());
        //This stops the motor at end of command or during a interupt
  }

  public Command reverseCommand() {
    return this.runEnd(()->setIntakeSpeed(-0.25), () -> m_intake.stopMotor())
    .withTimeout(0.2);
  }

  public Command stopIntake() {
    return this.runOnce(()-> m_intake.stopMotor());
  }

  public void setIntakeSpeed(double speed) {
    m_intake.set(speed);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double setpoint = SmartDashboard.getNumber("setpoint",0);
    
    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { shooterPIDController.setP(p); kP = p; }
    // if((i != kI)) { shooterPIDController.setI(i); kI = i; }
    // if((d != kD)) { shooterPIDController.setD(d); kD = d; }
    // if((ff != kFF)) { shooterPIDController.setFF(ff); kFF = ff; }
    // if((setpoint != kSetpoint)) { kSetpoint = setpoint;}
    
    // boolean noteState = SmartDashboard.getBoolean("IsNoteInIntake", false);
    // if((noteState != IsNoteInIntake)) { noteState = IsNoteInIntake;}
    SmartDashboard.putBoolean("IsNoteInIntake", IsNoteInIntake());
    SmartDashboard.putNumber("shooter volocity", -shooterVelocity());
  }
  
}
