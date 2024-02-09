// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.InOut;

public class IntakeCommand extends Command {
  private static InOut m_InOut;
  private boolean m_maunalOveride;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(InOut inOutSubsystem, Trigger trigger) {
    m_InOut = inOutSubsystem;
    m_maunalOveride = trigger.getAsBoolean();

    addRequirements(m_InOut);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      new RunCommand(() -> m_InOut.intakeNote(0.75, m_maunalOveride));
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return false;
  }
}
