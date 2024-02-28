// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InOut;

public class IntakeJoystickCommand extends Command {
  
  //private static final double kForwardScaleFactor = 1.0;
 
  private final InOut m_InOut;
  private final DoubleSupplier m_forwardSpeedSupplier;
  private final BooleanSupplier m_BooleanSupplier;

  /** Creates a new ArcadeDriveCommand. */
  public IntakeJoystickCommand(InOut inOut, DoubleSupplier forwardSpeedSupplier, BooleanSupplier disableJoystick) {
    m_InOut = inOut;
    m_forwardSpeedSupplier = forwardSpeedSupplier;
    m_BooleanSupplier = disableJoystick;
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_InOut);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_BooleanSupplier.getAsBoolean()) {
      //this disables the default command and uses buttons
      // do not put anything in here
    }
    else {
    m_InOut.setIntakeSpeed(m_forwardSpeedSupplier.getAsDouble());
    }
  }

  // Returns true when the command should end.
  @Override public boolean isFinished() {
    return false;
  }
}