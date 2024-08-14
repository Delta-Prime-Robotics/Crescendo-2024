// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.BreakIterator;
import java.util.Map;
import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.GamePad.Button;
import frc.robot.Constants.GamePad.LeftStick;
import frc.robot.Constants.GamePad.RightStick;
import frc.robot.commands.ArmManualMoveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.CalculateAndTurnCommand;
import frc.robot.commands.IntakeJoystickCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.InOut;
import frc.utils.SpeakerRotateUtil;

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
  private final SpeakerRotateUtil m_RotateUtil = new SpeakerRotateUtil(m_robotDrive);
  public boolean isAutonomous = true;
  
  // The driver's controller
  private final Joystick m_driverGamepad = new Joystick(Constants.UsbPort.kGamePadDr);
  //The Operator's controller
  private final Joystick m_operatorGamepad = new Joystick(Constants.UsbPort.kGamePadO);
  //The Programer's controller
  private final Joystick m_testingGampad = new Joystick(Constants.UsbPort.kTestingControler);
  
  private final SendableChooser<Command> m_AutoChooser = new SendableChooser<>();
  private final SendableChooser<Command> m_pathChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configurePathPlanerChooser();
    m_pathChooser = AutoBuilder.buildAutoChooser();
    
    // Configure the button bindings
    //configureAutonomousChooser();
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
    
    //hook bindings
    JoystickButton hookButtonLT = new JoystickButton(m_driverGamepad, Button.kX); 
    JoystickButton hookButtonRT = new JoystickButton(m_driverGamepad, Button.kB); 
    JoystickButton reverseTrigger = new JoystickButton(m_driverGamepad, Button.kRB);
    
    new JoystickButton(m_driverGamepad, Button.kRT)
      .onTrue(new RunCommand(() -> m_Hook.voidHookRun(1), m_Hook))
      .onFalse(new InstantCommand(() -> m_Hook.voidHookRun(0), m_Hook));

    new JoystickButton(m_driverGamepad, Button.kLT)
      .onTrue(new RunCommand(() -> m_Hook.voidHookRun(-1), m_Hook))
      .onFalse(new InstantCommand( () -> m_Hook.voidHookRun(0), m_Hook));
    
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
    
    new JoystickButton(m_testingGampad, Button.kA)
    .onTrue(m_InOut.intakeCommand(0.4))
    .onFalse(m_InOut.stopIntake());

    new JoystickButton(m_testingGampad, Button.kRT)
    .onTrue(m_Arm.getArmInGroundPostion())
    .onFalse(new InstantCommand(() -> m_Arm.armRun(0), m_Arm));

    // new JoystickButton(m_driverGamepad, Button.kX)
    // .whileTrue(new TurnToAngleCommand(
    //   m_RotateUtil
    //     .returnSpeakerAngle(
    //       () -> m_robotDrive.getPose().getX(),
    //       () -> m_robotDrive.getPose().getY()
    //       ),
    //   m_robotDrive));
    // new JoystickButton(m_driverGamepad, Button.kX)
    // .whileTrue(new ParallelRaceGroup(
    //     new RunCommand(() -> {
    //         // Update suppliers for robotX and robotY
    //         DoubleSupplier robotXSupplier = () -> m_robotDrive.getPose().getX();
    //         DoubleSupplier robotYSupplier = () -> m_robotDrive.getPose().getY();
            
    //         // Calculate and update the angle
    //         double angle = m_RotateUtil.returnSpeakerAngle(robotXSupplier, robotYSupplier);
    //         SmartDashboard.putNumber("Calculated Angle", angle); // Optional: Display the angle for debugging
    //     }, m_robotDrive),
        
    //     new TurnToAngleCommand(
    //         m_RotateUtil.returnSpeakerAngle(
    //             () -> m_robotDrive.getPose().getX(),
    //             () -> m_robotDrive.getPose().getY()
    //         ),
    //         m_robotDrive
    //     )
    // ));
    new JoystickButton(m_driverGamepad, Button.kX)
    .whileTrue(m_robotDrive.turnToAngleCommand(m_RotateUtil));
    // new JoystickButton(m_driverGamepad, Button.kX)
    // .onTrue(new InstantCommand(
    //   () -> m_RotateUtil.returnSpeakerAngle(
    //   () -> 9,
    //   () -> 3.5
    //   )));
    // new JoystickButton(m_testingGampad, Button.kY)
    // .onTrue(new InstantCommand(()->m_robotDrive.resetOdometry(
    //   new Pose2d(new Translation2d(0,0), 
    //   new Rotation2d(m_RotateUtil.returnSpeakerAngle(m_robotDrive)))
    //   )));

    new JoystickButton(m_driverGamepad, Button.kB)
    .onTrue(new InstantCommand(
      () -> m_robotDrive.resetOdometry(
        new Pose2d(
          new Random().nextDouble(16),
          new Random().nextDouble(8.2),
          new Rotation2d(new Random().nextDouble(360))
        )
      )
    ));
    
    // new JoystickButton(m_testingGampad, Button.kR)
    // .onTrue(m_InOut.intakeCommand(1))
    // .onFalse(m_InOut.stopIntake());

    // .onFalse(new InstantCommand( () -> m_InOut.setIntakeSpeed(.0),m_InOut)
    // .andThen(() -> m_InOut.m_bbLimitSwitch.enableLimitSwitch(false)));
  }

  // private void configureAutonomousChooser() {
  //   m_AutoChooser.setDefaultOption("Just Shoot", m_Autos.justShootCommand(m_Arm, m_InOut));
  //   m_AutoChooser.addOption("shoot and back", m_Autos.shootAndMoveCommand(m_Arm, m_InOut, m_robotDrive));
  //   m_AutoChooser.addOption("Do Nothing", m_Autos.doNothing());
  //   // m_AutoChooser.addOption("Back up", m_Autos.justBackUpCommand(m_robotDrive));
  //   // m_AutoChooser.addOption("Shoot Then Backup While Intaking", getAutonomousCommand());
  // }

  private void configurePathPlanerChooser(){
    NamedCommands.registerCommand("Shoot", m_Autos.justShootCommand(m_Arm, m_InOut));
    NamedCommands.registerCommand("SquatAndNomNom", m_Autos.toGroundAndGrabCommand(m_Arm, m_InOut));
    NamedCommands.registerCommand("ReverseNomNom", m_InOut.reverseCommand());
    NamedCommands.registerCommand("SquatAndNomNomReverse", m_Autos.toGroundAndGrabAndReverseCommand(m_Arm, m_InOut));
    NamedCommands.registerCommand("SpeakerAndSpinUp", m_Autos.speakerAndSpinUp(m_Arm, m_InOut));
    NamedCommands.registerCommand("SpinUpAndFeedNote", m_InOut.shootIntoSpeaker());
    NamedCommands.registerCommand("Speaker", m_Arm.armToSpeakerCommand());
    NamedCommands.registerCommand("FeedNote", m_InOut.intoShooter());
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
