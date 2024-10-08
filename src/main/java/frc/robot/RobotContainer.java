// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.BreakIterator;
import java.util.Map;
import java.util.Random;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.ArmManualMoveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeJoystickCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
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
  private final Autos m_Autos = new Autos();
  public boolean isAutonomous = true;

  
  // The driver's controller
  private final XboxController m_driverGamepad = new XboxController(Constants.UsbPort.kGamePadDr);
  //The Operator's controller
  private final XboxController m_operatorGamepad = new XboxController(Constants.UsbPort.kGamePadO);
  //The Programer's controller
  private final XboxController m_testingGampad = new XboxController(Constants.UsbPort.kTestingControler);
  
  private final SendableChooser<Command> m_AutoChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_pathChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configurePathPlanerChooser();

    m_pathChooser = AutoBuilder.buildAutoChooser();
    
    // Configure the button bindings
    configureAutonomousChooser();
    configureBindings();

    SmartDashboard.putData("PathPlaner Chooser", m_pathChooser);
 
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
            -MathUtil.applyDeadband(m_driverGamepad.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverGamepad.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverGamepad.getRightX(), OIConstants.kDriveDeadband),
            true, true),
        m_robotDrive));
      
    if (m_Arm != null && m_operatorGamepad != null) {
        m_Arm.setDefaultCommand(new ArmManualMoveCommand(m_Arm, 
        () -> -MathUtil.applyDeadband(m_operatorGamepad.getLeftY(), 0.05)
      ));
    }
    
    if (m_InOut != null && m_operatorGamepad != null) {
        m_InOut.setDefaultCommand(new IntakeJoystickCommand (m_InOut, 
        () -> -MathUtil.applyDeadband(m_operatorGamepad.getRightY(), 0.05),
        () -> //pass a true when you want to use the intake in a command
        isAutonomous //RobotModeTriggers.autonomous().getAsBoolean();
        )); 
    }

    if (m_Hook != null && m_driverGamepad != null) {
        m_Hook.setDefaultCommand(
          new RunCommand( 
            ()->m_Hook.runBothHooks(
              () -> -MathUtil.applyDeadband(m_driverGamepad.getLeftTriggerAxis(), 0.05),
              () -> -MathUtil.applyDeadband(m_driverGamepad.getRightTriggerAxis(),0.05),
              m_driverGamepad.getYButton())
              , m_Hook));
    }


    new JoystickButton(m_driverGamepad, Button.kBack.value)
      .onTrue(new InstantCommand(
        () -> m_robotDrive.zeroHeading(),
        m_robotDrive));

    // new JoystickButton(m_operatorGamepad, Button.kLT)
    // .onTrue(m_InOut.shootIntoSpeaker());
    
    // new JoystickButton(m_operatorGamepad, XboxController.Button.kX.value)
    // .onTrue(m_InOut.intoShooter());

    new JoystickButton(m_operatorGamepad, Button.kB.value)
    .onTrue(new InstantCommand(() -> m_InOut.setShooterRef(InOut.kSetpoint)))
    .onFalse(new InstantCommand(()-> m_InOut.setShooterRef(0)));

    new JoystickButton(m_operatorGamepad, Button.kRightBumper.value)
    .onTrue(new RunCommand(() -> m_Arm.getArmInPositionSpeaker(), m_Arm))
    .onFalse(new InstantCommand( () -> m_Arm.armRun(0), m_Arm));

    new JoystickButton(m_operatorGamepad, Button.kA.value)
    .onTrue(m_InOut.autoIntakeCommand())
    .onFalse(m_InOut.stopIntake());

    new JoystickButton(m_operatorGamepad, Button.kX.value)
    .whileTrue(m_InOut.intoShooter());

    new JoystickButton(m_driverGamepad, Button.kRightBumper.value)
    .onTrue(new RunCommand(() -> m_Hook.voidHookRun(1), m_Hook))
    .onFalse(new InstantCommand(() -> m_Hook.voidHookRun(0), m_Hook));

  new JoystickButton(m_driverGamepad, Button.kLeftBumper.value)
    .onTrue(new RunCommand(() -> m_Hook.voidHookRun(-1), m_Hook))
    .onFalse(new InstantCommand( () -> m_Hook.voidHookRun(0), m_Hook));


    new JoystickButton(m_testingGampad, Button.kLeftBumper.value)
    .onTrue(m_Arm.getArmInGroundPostion())
    .onFalse(new InstantCommand( () -> m_Arm.armRun(0), m_Arm));

    new JoystickButton(m_testingGampad, Button.kA.value)
    .onTrue(m_Autos.speakerAndSpinUp(m_Arm, m_InOut))
    .onFalse(new InstantCommand( () -> m_Arm.armRun(0), m_Arm)); 

    new JoystickButton(m_testingGampad, Button.kRightBumper.value)
    .onTrue(m_Arm.getArmInGroundPostion())
    .onFalse(new InstantCommand(() -> m_Arm.armRun(0), m_Arm));


    new JoystickButton(m_testingGampad, Button.kX.value)
    .onTrue(new InstantCommand(
      () -> m_robotDrive.resetOdometry(
        new Pose2d(
          new Random().nextDouble(16),
          new Random().nextDouble(8.2),
          new Rotation2d(new Random().nextDouble(360))
        )
      )
    ));

    // new JoystickButton(m_operatorGamepad, Button.kRB)
    // .onTrue(new PrintCommand("arm angle  " + String.format("%2.5", m_Arm.armRotation()))
    // .andThen(new PrintCommand("heading" + String.format("%2.5", m_robotDrive.getHeading().getDegrees()))));

    // new JoystickButton(m_operatorGamepad, Button.kRB)
    // .onTrue(new RunCommand(() -> m_InOut.m_bbLimitSwitch.enableLimitSwitch(true), m_InOut)
    //   .alongWith( new ParallelDeadlineGroup(
    //           new WaitCommand(.5), 
    //           new RunCommand(() -> m_InOut.setIntakeSpeed(.45))
    // )))
    // .onFalse(new InstantCommand( () -> m_InOut.setIntakeSpeed(.0),m_InOut)
    // .andThen(() -> m_InOut.m_bbLimitSwitch.enableLimitSwitch(false)));
    
    // //hook bindings
    // JoystickButton hookButtonLT = new JoystickButton(m_driverGamepad, Button.kX.value); 
    // JoystickButton hookButtonRT = new JoystickButton(m_driverGamepad, Button.kB.value); 
    // JoystickButton reverseTrigger = new JoystickButton(m_driverGamepad, Button.kRightBumper.value);
    
    // hookButtonLT.whileTrue(
    //   new ConditionalCommand(
    //     m_Hook.leftHookRunCommand(false),
    //     m_Hook.leftHookRunCommand(true),
    //     reverseTrigger
    // ));
    
    // hookButtonRT.whileTrue(
    //   new ConditionalCommand(
    //     m_Hook.rightHookRunCommand(false),
    //     m_Hook.rightHookRunCommand(true),
    //     reverseTrigger
    // ));

    // new JoystickButton(m_testingGampad, Button.kRT)
    // .onTrue(m_Arm.getArmInGroundPostion())
    // .onFalse(new InstantCommand( () -> m_Arm.armRun(0), m_Arm));

    // new JoystickButton(m_testingGampad, Button.kB)
    // .onTrue(m_Autos.speakerAndSpinUp(m_Arm, m_InOut))
    // .onFalse(new InstantCommand( () -> m_Arm.armRun(0), m_Arm));
    
    // new JoystickButton(m_testingGampad, Button.kR)
    // .onTrue(m_InOut.intakeCommand(1))
    // .onFalse(m_InOut.stopIntake());
  }

  private void configureAutonomousChooser() {
    m_AutoChooser.setDefaultOption("Just Shoot", m_Autos.justShootCommand(m_Arm, m_InOut));
    m_AutoChooser.addOption("shoot and back", m_Autos.shootAndMoveCommand(m_Arm, m_InOut, m_robotDrive));
    m_AutoChooser.addOption("Do Nothing", m_Autos.doNothing());
    // m_AutoChooser.addOption("Back up", m_Autos.justBackUpCommand(m_robotDrive));
    // m_AutoChooser.addOption("Shoot Then Backup While Intaking", getAutonomousCommand());
  }

  private void configurePathPlanerChooser(){
    NamedCommands.registerCommand("Shoot", m_Autos.justShootCommand(m_Arm, m_InOut));
    NamedCommands.registerCommand("SquatAndNomNom", m_Autos.toGroundAndGrabCommand(m_Arm, m_InOut));
    NamedCommands.registerCommand("ReverseNomNom", m_InOut.reverseCommand());
    NamedCommands.registerCommand("SquatAndNomNomReverse", m_Autos.toGroundAndGrabAndReverseCommand(m_Arm, m_InOut));
    NamedCommands.registerCommand("SpeakerAndSpinUp", m_Autos.speakerAndSpinUp(m_Arm, m_InOut));
    NamedCommands.registerCommand("SpinUpAndFeedNote", m_InOut.shootIntoSpeaker());
    NamedCommands.registerCommand("Speaker", m_Arm.armToSpeakerCommand());
    NamedCommands.registerCommand("FeedNote", m_InOut.intoShooter());
    NamedCommands.registerCommand("AutoNomNom", m_Autos.toGroundAndAutoGrabCommand(m_Arm, m_InOut));
    NamedCommands.registerCommand("NomNomWhileShoot", m_Autos.intakeWhileShooting(m_InOut));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_pathChooser.getSelected();
    //return m_AutoChooser.getSelected();
    
  }
}
