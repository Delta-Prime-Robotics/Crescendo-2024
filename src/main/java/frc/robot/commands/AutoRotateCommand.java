// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRotateCommand{
  private final DriveSubsystem m_drive;
  // Robot's current position and angle
  double robotX;  
  double robotY;  
  double robotAngleRadians; 
  double targetX;
  final double targetY = 5.55; 
  double targetAngleRadians;

  double angleDifference;

  //PID turing Control
  static final PIDController turningControler = 
  new PIDController(
    0.1,
    0,
    0
  );


  /** Creates a new AutoRotateCommand. 
   * @param m_drive */
  public AutoRotateCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = driveSubsystem;
    m_drive.resetOdometry(new Pose2d(robotX, robotY, new Rotation2d()));
    turningControler.enableContinuousInput(-1 * Math.PI, Math.PI);
    turningControler.setTolerance(0.0349066, 0.1);
    robotX = m_drive.getPose().getX();
    robotY = m_drive.getPose().getY();
    robotAngleRadians = m_drive.getPose().getRotation().getRadians();
    if (onRedSide()) {
      targetX = 16.54;
    } else {
      targetX = 0.0;
    }
    // Calculate the angle to turn to
    targetAngleRadians = calculateAngle(robotX, robotY, targetX, targetY);
    angleDifference = calculateAngleDifference(robotAngleRadians, targetAngleRadians);
  }

  public boolean onRedSide(){
    var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
    return false;
  }

  // Function to calculate the angle between two points
  public static double calculateAngle(double currentX, double currentY, double targetX, double targetY) {
    double angle = Math.atan2(targetY - currentY, targetX - currentX);
    while (angle > Math.PI) {
      angle -= 2 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2 * Math.PI;
    }
    return angle;
  }

  // Function to calculate the angle in angle
  public static double calculateAngleDifference(double currentAngle, double targetAngle) {
    // Ensure the angle is within -π to π range
    double difference = targetAngle - currentAngle;
    while (difference > Math.PI) {
      difference -= 2 * Math.PI;
    }
    while (difference < -Math.PI) {
      difference += 2 * Math.PI;
    }
    return difference;
  }

  public Command turnCommand(){
   return new PIDCommand(
        turningControler, 
        () -> robotAngleRadians, 
        targetAngleRadians, 
        output -> m_drive.drive(
            0,
            0,
            output, 
            true,
            true
        ),
        m_drive).until(()-> robotAngleRadians == targetAngleRadians);
    
  }
  

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() { 
  // }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   // Calculate the angle angle
  //m_drive.resetOdometry(new Pose2d(robotX,robotY, new Rotation2d(targetAngleRadians)));
    //System.out.println("Target angle: " + Math.toDegrees(targetAngleRadians) + " degrees");
    //System.out.println("Angle difrence: " + Math.toDegrees(angleDifference) + " degrees");
  // // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  // }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
