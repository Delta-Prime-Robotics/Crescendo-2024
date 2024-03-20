// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.BreakIterator;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GamePad.Button;
import frc.robot.Constants.GamePad.LeftStick;
import frc.robot.Constants.GamePad.RightStick;
import frc.robot.commands.ArmManualMoveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeJoystickCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HookSubsystem;
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
  private final HookSubsystem m_Hook = new HookSubsystem();
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
        isAutonomous //RobotModeTriggers.autonomous().getAsBoolean();
        ));
      
    }

    new JoystickButton(m_driverGamepad, Button.kBack)
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

      // new JoystickButton(m_operatorGamepad, Button.kRB)
      // .onTrue(new RunCommand(() -> m_InOut.m_bbLimitSwitch.enableLimitSwitch(true), m_InOut)
      //   .alongWith( new ParallelDeadlineGroup(
      //           new WaitCommand(.5), 
      //           new RunCommand(() -> m_InOut.setIntakeSpeed(.45))
      // )))
      // .onFalse(new InstantCommand( () -> m_InOut.setIntakeSpeed(.0),m_InOut)
      // .andThen(() -> m_InOut.m_bbLimitSwitch.enableLimitSwitch(false)));
    
    //hook bindings
    JoystickButton hookButtonLT = new JoystickButton(m_driverGamepad, Button.kX); 
    JoystickButton hookButtonRT = new JoystickButton(m_driverGamepad, Button.kB); 
    JoystickButton reverseTrigger = new JoystickButton(m_driverGamepad, Button.kRB);
    
    new JoystickButton(m_driverGamepad, Button.kLT)
      .onTrue(new RunCommand(() -> m_Hook.voidHookRun(1), m_Hook))
      .onFalse(new InstantCommand(() -> m_Hook.voidHookRun(0), m_Hook));

    new JoystickButton(m_driverGamepad, Button.kRT)
      .onTrue(new RunCommand(() -> m_Hook.voidHookRun(-1), m_Hook))
      .onFalse(new InstantCommand( () -> m_Hook.voidHookRun(0), m_Hook));
    
    hookButtonLT.whileTrue(
      new ConditionalCommand(
        m_Hook.leftHookRunCommand(false),
        m_Hook.leftHookRunCommand(true),
        reverseTrigger
    ));
    
    hookButtonRT.whileTrue(
      new ConditionalCommand(
        m_Hook.rightHookRunCommand(false),
        m_Hook.rightHookRunCommand(true),
        reverseTrigger
    ));
  }

  private void configureAutonomousChooser() {
    m_AutoChooser.setDefaultOption("Do Nothing", m_Autos.doNothing());
    m_AutoChooser.addOption("Back up", m_Autos.justBackUpCommand(m_robotDrive));
    m_AutoChooser.addOption("Just Shoot", m_Autos.justShootCommand(m_Arm, m_InOut));
    m_AutoChooser.addOption("shoot and back", m_Autos.shootAndMoveCommand(m_Arm, m_InOut, m_robotDrive));
    m_AutoChooser.addOption("really Just Back Up", m_Autos.reallyJustBackUpCommand(m_robotDrive));
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
