// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.InOut;

public class IntakeCommand extends Command {
  private static InOut m_InOut;

  /** Creates a new IntakeCommand. */
  public IntakeCommand(InOut inOutSubsystem) {
    m_InOut = inOutSubsystem;
    
    addRequirements(m_InOut);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //will the setIntake finish the ParallelDeadlineGroup before the WaitUntilCommand?    // new SequentialCommandGroup(
    // new ParallelDeadlineGroup(
    //   new WaitUntilCommand(m_InOut.isNoteInIntake()),
    //   new RunCommand(() -> m_InOut.setIntake(0.75), m_InOut)
    // )
    // .andThen(() -> m_InOut.setIntake(0))
    // );

    new ParallelDeadlineGroup(
      new WaitUntilCommand(m_InOut.isNoteInIntake()),
      new RunCommand(() -> m_InOut.setIntake(0.75), m_InOut));
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_InOut.setIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return false;
  }
}
