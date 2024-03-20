// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;
import frc.robot.Constants.NeoMotorConstants;

public class HookSubsystem extends SubsystemBase {

  //Setting arms and Encoder
  private final CANSparkMax m_left; //left arm
  private final CANSparkMax m_right; //right arm

  /** Creates a new ArmSubsystem. */
  public HookSubsystem() {
    m_left = new CANSparkMax(HookConstants.kHookLeftCanId, MotorType.kBrushless);
    m_right = new CANSparkMax(HookConstants.kHookRightCanId, MotorType.kBrushless);
    
    //makeing right hook inverted
    m_right.setInverted(true);
    
    //set smartCurrentLimits
    m_left.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_right.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    //Set IdleMode's
    m_left.setIdleMode(HookConstants.kHookIdleMode);
    m_right.setIdleMode(HookConstants.kHookIdleMode);
  }

  private void leftRun(double speed) {
    m_left.set(speed);
  }

  private void rightRun(double speed) {
    m_right.set(speed);
  }

  public void voidHookRun(double speed){
    m_left.set(speed);
    m_right.set(speed);
  }

  /**
   * @param direction true = forwards false = backwards
   * @return runs left Hook
   */
  public Command leftHookRunCommand(boolean direction) {
    double speed = 0.75;
    return this.startEnd(
      direction  
      ? () -> leftRun(speed) 
      : () -> leftRun(-speed),
      ()-> m_left.stopMotor()
      // ? ()-> System.out.println("left forwards")   
      // : ()-> System.out.println("left backwards"), 
      // ()-> System.out.println("left stop")        
    );
  }
  /**
   * @param direction true = forwards false = backwards
   * @return runs right Hook
   */
  public Command rightHookRunCommand(boolean direction) {
    double speed = 0.75;
    return this.startEnd(
      direction
      ? () -> rightRun(speed) 
      : () -> rightRun(-speed),
      () -> m_right.stopMotor()
      // ? ()-> System.out.println("right forwards")   
      // : ()-> System.out.println("right backwards"), 
      // ()-> System.out.println("right stop")        
    );
  }

  public Command runBothAsCommandTest() {
    return this.startEnd(
      () -> voidHookRun(0.5), 
      () -> m_left.stopMotor()
    );
  }
  

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
