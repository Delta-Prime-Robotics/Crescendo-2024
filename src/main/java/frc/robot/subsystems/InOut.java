// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.InOutConstants;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class InOut extends SubsystemBase {
  //Output Top & Bottom 
  private final CANSparkMax m_bottomSparkMax;
  private final CANSparkMax m_topSparkMax;
  private final CANSparkMax m_intakeSparkMax;

  
  //IR Beam Break
  public static DigitalInput bbInput = new DigitalInput(0);

  /** Creates a new InOut. */
  public InOut() {
    //setting CAN ID's for Output Motor Controlers
    m_intakeSparkMax = new CANSparkMax(InOutConstants.kIntakeCanId, MotorType.kBrushless);
    m_bottomSparkMax = new CANSparkMax(InOutConstants.kBottomOutputCanId, MotorType.kBrushless);


    m_topSparkMax = new CANSparkMax(InOutConstants.kTopOutputCanId, MotorType.kBrushless);
  }

  public static boolean getBB() {
    return bbInput.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
