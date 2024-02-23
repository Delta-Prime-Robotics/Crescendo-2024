// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.InOut;
import frc.robot.subsystems.ArmSubsystem.ArmState;

import java.util.Map;
import java.util.function.Supplier;

public class ShooterAmpOrSpeakerCommand extends Command {
  private static ArmSubsystem m_arm;
  private static InOut m_InOut;

  /** Creates a new ShooterAmpOrSpeakerCommand. */
  public ShooterAmpOrSpeakerCommand(ArmSubsystem armSubsystem, InOut inOutSubsystem) {
    m_arm = armSubsystem;
    m_InOut = inOutSubsystem;
  }

  private final Command commandToRun =
    new SelectCommand<>(
      Map.ofEntries(
        Map.entry(ArmState.AMP, new PrintCommand("AMP POSITION")), 
        Map.entry(ArmState.SPEAKER, new PrintCommand("SPEAKER POSITION")),
        Map.entry(ArmState.GROUND, new PrintCommand("GROUND POSITION")),
        Map.entry(ArmState.ERECT, new PrintCommand("ERECT POSITION"))
        ), m_arm.armStateLogic);  
  
   @Override
  public void initialize() {
    commandToRun.schedule();
  }

  @Override
  public boolean isFinished() {
    return commandToRun.isFinished();
  }
}
