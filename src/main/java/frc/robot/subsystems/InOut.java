// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import java.util.function.BooleanSupplier;

import frc.robot.Constants.InOutConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
public class InOut extends SubsystemBase {
  //Output Top & Bottom 
  private final CANSparkMax m_bottomSparkMax;
  private final CANSparkMax m_topSparkMax;
  private final CANSparkMax m_intakeSparkMax;

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  static SimpleMotorFeedforward feedforwardShooter = new SimpleMotorFeedforward(0, 0, 0);
  //static PIDController shooterPID = new PIDController(0, 0, 0);
  
  //IR Beam Break
  public static DigitalInput bbInput = new DigitalInput(0);

  /** Creates a new InOut. */
  public InOut() {
    //setting CAN ID's for Shooter Motor Controlers
    m_bottomSparkMax = new CANSparkMax(InOutConstants.kBottomOutputCanId, MotorType.kBrushless);
    m_topSparkMax = new CANSparkMax(InOutConstants.kTopOutputCanId, MotorType.kBrushless);
    m_topSparkMax.follow(m_bottomSparkMax);
    
    //invert
    m_topSparkMax.setInverted(true);
    m_bottomSparkMax.setInverted(true);

    //seting idle to coast
    m_bottomSparkMax.setIdleMode(InOutConstants.kShooterIdleMode);
    m_topSparkMax.setIdleMode(InOutConstants.kShooterIdleMode);

    //setting CAN ID's for Intake Motor Controler
    m_intakeSparkMax = new CANSparkMax(InOutConstants.kIntakeCanId, MotorType.kBrushless);
    
    //setting idle to break
    m_intakeSparkMax.setIdleMode(InOutConstants.kIntakeIdleMode);
  }

  public BooleanSupplier isNoteInIntake() {
    //when the BeamBreak is false there is a note in the Intake
    if (bbInput.get() == false) {
      return () -> true; 
    }
    else{
    return () -> false;
    }
  }
  
  public static Boolean getBBstate() {
    return bbInput.get();
  }

  public void setShooter(double velocity, double accel ){
    m_topSparkMax.setVoltage(feedforwardShooter.calculate(velocity, accel));
  }

  public void setIntake(double speed){
    double absSpeed = Math.abs(speed);
    //checks if speed is greater than 1 or -1
    if (1 < absSpeed){
    m_intakeSparkMax.set(absSpeed);
    }
    else {
    m_intakeSparkMax.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
