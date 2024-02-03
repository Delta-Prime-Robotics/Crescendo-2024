// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import java.util.function.BooleanSupplier;

import frc.robot.Constants;
import frc.robot.Constants.InOutConstants;
import frc.robot.Constants.NeoMotorConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
public class InOut extends SubsystemBase {
  //Output Top & Bottom 
  private final CANSparkMax m_FollowerShooter;
  private final CANSparkMax m_LeaderShooter;
  private final CANSparkMax m_intakeSparkMax;

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  static SimpleMotorFeedforward feedforwardShooter = new SimpleMotorFeedforward(0, 0, 0);
  //static PIDController shooterPID = new PIDController(1,0, 1);//keep ki as zero or as best you can
  
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

  //checks if speed is greater than 1 or -1
  //returns either a abs or unchange vaulef
  public double speedCheck(double speed, Boolean abs) {
    double absSpeed = Math.abs(speed);
    double result = 0;
    if (1 < absSpeed){
      result = abs ? absSpeed : speed; //I forgot why I added this 
    }
    return result;
  }

  public static boolean m_Hitting = false;
  //Shooter set speed
  public void setShooter(double speed){
    //speed = speedCheck(speed, false);
    m_LeaderShooter.set(speed);
    //m_FollowerShooter.set(speed);
    m_Hitting = !m_Hitting;
  }
  
  //work in progress
  public void setShooterPID(double velocity, double accel ){
    m_LeaderShooter.setVoltage(feedforwardShooter.calculate(velocity, accel));
  }
 

  // intake only needs to forwards so the speed will always be positive
  public void intakeNote(double speed){
    if (isNoteInIntake().getAsBoolean()) {
      m_intakeSparkMax.set(0);
    }
    else{
      m_intakeSparkMax.set(speed);
    }
    //speed = speedCheck(speed, true);
  }

  public InstantCommand intakeStop(){
    return new InstantCommand(() -> m_intakeSparkMax.set(0));
  }


  public Command intoShooter(){
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitUntilCommand(isNoteOutOfIntake()),
        new RunCommand(() -> intakeNote(0.1))
      )
      .andThen(intakeStop())
    );
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
