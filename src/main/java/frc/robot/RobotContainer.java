// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePad;
import frc.robot.Constants.GamePad.Button;
import frc.robot.Constants.GamePad.LeftStick;
import frc.robot.Constants.GamePad.RightStick;
import frc.robot.commands.ArmManualMoveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeJoystickCommand;
import frc.robot.commands.ShooterAmpOrSpeakerCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InOut;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(); 
  private final InOut m_InOut = new InOut();
  private final ArmSubsystem m_Arm = new ArmSubsystem();
  private final Dashboard m_Dashboard = new Dashboard(m_robotDrive, m_InOut, m_Arm);
  private final Autos m_Autos = new Autos();
  public boolean isAutonomous = true;
  
  // The driver's controller
  private final Joystick m_driverGamepad = new Joystick(Constants.UsbPort.kGamePadDr);
  //The Operator's controller
  private final Joystick m_operatorGamepad = new Joystick(Constants.UsbPort.kGamePadO);
  
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Configure the button bindings
    configureBindings();
    configureAutonomousChooser();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureBindings() {
        // //swerve Drive
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
     new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverGamepad.getRawAxis(LeftStick.kUpDown), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverGamepad.getRawAxis(LeftStick.kLeftRight), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverGamepad.getRawAxis(RightStick.kLeftRight), OIConstants.kDriveDeadband),
              true, true),
          m_robotDrive));
      
      if (m_Arm != null && m_operatorGamepad != null) {
          m_Arm.setDefaultCommand(new ArmManualMoveCommand(m_Arm, 
          () -> -MathUtil.applyDeadband(m_operatorGamepad.getRawAxis(LeftStick.kUpDown), 0.05)
        ));
      }
    
      if (m_InOut != null && m_operatorGamepad != null) {
          m_InOut.setDefaultCommand(new IntakeJoystickCommand (m_InOut, 
          () -> -MathUtil.applyDeadband(m_operatorGamepad.getRawAxis(RightStick.kUpDown), 0.05),
          () -> //pass a true when you want to use the intake in a command
          isAutonomous
          ));
        
      }

     new JoystickButton(m_driverGamepad, Button.kLB)
         .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    
     new JoystickButton(m_operatorGamepad, Button.kLT)
    .onTrue(m_InOut.shootIntoSpeaker());
    
    new JoystickButton(m_operatorGamepad, Button.kX)
    .onTrue(m_InOut.intoShooter());

    new JoystickButton(m_operatorGamepad, Button.kB)
    .onTrue(new InstantCommand(() -> m_InOut.setShooterRef(InOut.kSetpoint)))
    .onFalse(new InstantCommand(()-> m_InOut.setShooterRef(0)));

    new JoystickButton(m_operatorGamepad, Button.kRT)
      .onTrue(new RunCommand(() -> m_Arm.getArmInPositionSpeaker(), m_Arm))
      .onFalse(new InstantCommand( () -> m_Arm.armRun(0), m_Arm));

      new JoystickButton(m_operatorGamepad, Button.kRB)
      .onTrue(new RunCommand(() -> m_InOut.intakeCommand(.9), m_InOut))
      .onFalse(new InstantCommand( () -> m_InOut.m_bbLimitSwitch.enableLimitSwitch(false), m_Arm));
      
  }

  private void configureAutonomousChooser() {
    m_AutoChooser.setDefaultOption("Do Nothing", m_Autos.doNothing());
    m_AutoChooser.addOption("Back up", m_Autos.justBackUpCommand(m_robotDrive));
    m_AutoChooser.addOption("Just Shoot", m_Autos.justShootCommand(m_Arm, m_InOut));
    m_AutoChooser.addOption("shoot and back", m_Autos.shootAndMoveCommand(m_Arm, m_InOut, m_robotDrive));
    
    SmartDashboard.putData(m_AutoChooser);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_AutoChooser.getSelected();
  }
}
