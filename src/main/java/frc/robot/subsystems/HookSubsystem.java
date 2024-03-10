// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;
import frc.robot.Constants.NeoMotorConstants;

public class HookSubsystem extends SubsystemBase {

  //Setting arms and Encoder
  private final CANSparkMax m_leader; //left arm
  private final CANSparkMax m_follower; //right arm

  /** Creates a new ArmSubsystem. */
  public HookSubsystem() {
    m_leader = new CANSparkMax(HookConstants.kHookLeftCanId, MotorType.kBrushless);
    m_follower = new CANSparkMax(HookConstants.kHookRightCanId, MotorType.kBrushless);
    
    //makeing left arm the leader
    m_follower.follow(m_leader, true);
    //set smartCurrentLimits
    m_leader.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    m_follower.setSmartCurrentLimit(NeoMotorConstants.kNeoSetCurrent);
    //Set IdleMode's
    m_leader.setIdleMode(HookConstants.kHookIdleMode);
    m_follower.setIdleMode(HookConstants.kHookIdleMode);

  }

  public void HookRun(double speed){
      m_leader.set(speed);
  }
  
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
