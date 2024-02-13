// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.BooleanSupplier;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.InOutConstants;
import frc.robot.Constants.NeoMotorConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class InOut extends SubsystemBase {
 
  //Output Top & Bottom 
  private final CANSparkMax m_FollowerShooter;
  private final CANSparkMax m_LeaderShooter;
  private SparkPIDController shooterPIDController;
  private final CANSparkMax m_intakeSparkMax;

  private static double kFF = 1;
  private static double kP = 0;
  private static double kI = 0;
  private static double kD = 0;
  
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

    //setting CAN ID's for Intake Motor Controler
    m_intakeSparkMax = new CANSparkMax(InOutConstants.kIntakeCanId, MotorType.kBrushless);
    m_intakeSparkMax.setIdleMode(InOutConstants.kIntakeIdleMode);
    m_intakeSparkMax.setSmartCurrentLimit(NeoMotorConstants.kNeo550SetCurrent);
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
  public void setShooter(double speed){
    this.shooterPIDController.setReference(speed, ControlType.kVelocity); //Speed is in RPMs
  }


  public void intakeNote(double speed, boolean maunalOveride){
    if (maunalOveride){
      m_intakeSparkMax.set(speed);
    }
    else if (isNoteInIntake().getAsBoolean()) {
      m_intakeSparkMax.set(0);
    }
    else{
      m_intakeSparkMax.set(speed);
    }
  }

  public InstantCommand intakeStop(){
    return new InstantCommand(() -> m_intakeSparkMax.set(0));
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { shooterPIDController.setP(p); kP = p; }
    if((i != kI)) { shooterPIDController.setI(i); kI = i; }
    if((d != kD)) { shooterPIDController.setD(d); kD = d; }
    if((ff != kFF)) { shooterPIDController.setFF(ff); kFF = ff; }

  }
}
