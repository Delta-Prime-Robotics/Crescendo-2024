// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.InOut;


/** Add your docs here. */
public final class Autos {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    
    public final Command justBackUpCommand(DriveSubsystem m_drive){
    return new RunCommand(
        ()-> m_drive.drive(
            -0.5,
            0,
            0,
            true,
            true)
    , m_drive)
    .withTimeout(0.5);
    }

    public final Command intakeWhileShooting(InOut m_InOut){
        return new ParallelCommandGroup(
            m_InOut.intakeCommand(1),
            new RunCommand(() -> m_InOut.setShooterRef(1500))  
        );
        
    }

    public final Command backupWithGroundAndGrabCommand(ArmSubsystem m_Arm, InOut m_InOut, DriveSubsystem m_drive){
        return new ParallelCommandGroup( 
            m_Arm.getArmInGroundPostion(),
            m_InOut.intakeCommand(1)
                .withTimeout(3),
            justBackUpCommand(m_drive)
                .beforeStarting(Commands.waitSeconds(0.5))
        );
    
    }
   
    public final Command toGroundAndGrabAndReverseCommand(ArmSubsystem m_Arm, InOut m_InOut){
        return m_Arm.getArmInGroundPostion()
        .alongWith(m_InOut.intakeCommand(1))
        .withTimeout(2)
        .andThen(m_InOut.reverseCommand());
    }

    public final Command toGroundAndGrabCommand(ArmSubsystem m_Arm, InOut m_InOut){
        return m_Arm.getArmInGroundPostion()
        .alongWith(m_InOut.intakeCommand(1))
        .withTimeout(2);
    }
    
    public final Command toGroundAndAutoGrabCommand(ArmSubsystem m_Arm, InOut m_InOut){
        return m_Arm.getArmInGroundPostion()
        .alongWith(m_InOut.autoIntakeCommand());
    }
    
    public final Command justShootCommand(ArmSubsystem m_Arm, InOut m_InOut) {
        return new RunCommand(() -> m_Arm.getArmInPositionSpeaker(), m_Arm)
        .until(()-> m_Arm.armAngleInSpeakerRange())
        .andThen(m_InOut.shootIntoSpeaker())
        .andThen(new InstantCommand(() -> m_Arm.armRun(0), m_Arm));
    }

    public final Command speakerAndSpinUp(ArmSubsystem m_Arm, InOut m_InOut) {
        return m_Arm.armToSpeakerCommand()
        .andThen(m_InOut.spinUpShooter());
    }	     


    
    public final Command shootAndMoveCommand(ArmSubsystem m_Arm, InOut m_InOut, DriveSubsystem m_robotDrive){
        // // An example trajectory to follow. All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(15.13, 5.00, new Rotation2d(0)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(new Translation2d(1.0, 5.50)), //, new Translation2d(1.5, 0)
        //     // End 3 meters straight ahead of where we started, facing forward
        //     new Pose2d(14.30, 5.50, new Rotation2d(0)),
        //     config);

        // var thetaController = new ProfiledPIDController(
        //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        // thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //     exampleTrajectory,
        //     m_robotDrive::getPose, // Functional interface to feed supplier
        //     DriveConstants.kDriveKinematics,

        //     // Position controllers
        //     new PIDController(AutoConstants.kPXController, 0, 0),
        //     new PIDController(AutoConstants.kPYController, 0, 0),
        //     thetaController,
        //     m_robotDrive::setModuleStates,
        //     m_robotDrive);

        return justShootCommand(m_Arm,m_InOut)
        .andThen(justBackUpCommand(m_robotDrive))
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }

    public final Command shootAndIntakeWhileBackupCommand(ArmSubsystem m_Arm, InOut m_InOut, DriveSubsystem m_drive){
        return justShootCommand(m_Arm, m_InOut)
        .andThen(backupWithGroundAndGrabCommand(m_Arm, m_InOut, m_drive));
    }

    public final Command doNothing() {
        return null;
    }
}
