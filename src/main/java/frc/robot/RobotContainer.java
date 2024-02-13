// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GamePad;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InOut;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.List;

import com.fasterxml.jackson.databind.introspect.ConcreteBeanPropertyBase;

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
  

  
  // The driver's controller
  private final Joystick m_driverGamepad = new Joystick(Constants.UsbPort.kGamePadDr);
  //The Operator's controller
  private final Joystick m_operatorGamepad = new Joystick(Constants.UsbPort.kGamePadO);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
    
    m_Dashboard.register();
    
  }
  // Configure default commands
  private void configureDefaultCommands() {
    m_robotDrive.setDefaultCommand(
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
   new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverGamepad.getRawAxis(Constants.GamePad.LeftStick.kUpDown), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverGamepad.getRawAxis(Constants.GamePad.LeftStick.kLeftRight), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverGamepad.getRawAxis(Constants.GamePad.RightStick.kLeftRight), OIConstants.kDriveDeadband),
            true, true),
        m_robotDrive));
    
    //Manual Arm
    if (m_Arm != null && m_operatorGamepad != null) {
      m_Arm.setDefaultCommand(new ArmMoveCommand(m_Arm, 
      () -> -m_operatorGamepad.getRawAxis(GamePad.LeftStick.kUpDown) 
      ));
    }


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
  private void configureButtonBindings() {
    new JoystickButton(m_driverGamepad, GamePad.Button.kLB)
        .toggleOnTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    //Intake Comand
    Trigger intakeMaunalOveride = new JoystickButton(m_operatorGamepad,GamePad.Button.kY);

    new JoystickButton(m_operatorGamepad, GamePad.Button.kA)
      .onTrue(new RunCommand(
        () -> m_InOut.intakeNote(1, intakeMaunalOveride.getAsBoolean()), m_InOut
      ))
      .onFalse(new InstantCommand(
        () -> m_InOut.intakeNote(0,true), m_InOut
      ));

    //Shooter Comand
    new JoystickButton(m_operatorGamepad, GamePad.Button.kB)
    .onTrue(new RunCommand(() -> m_InOut.setShooter(1), m_InOut))
    .onFalse(new InstantCommand(() -> m_InOut.setShooter(0), m_InOut));
    
    //arm command
    new JoystickButton(m_operatorGamepad, GamePad.Button.kX)
    .onTrue(new RunCommand(() -> m_Arm.armRun(0.5), m_Arm))
    .onFalse(new InstantCommand(() -> m_Arm.armRun(0), m_Arm));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
