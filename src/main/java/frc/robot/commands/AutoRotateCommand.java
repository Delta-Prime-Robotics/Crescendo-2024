// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRotateCommand extends Command {
  private final DriveSubsystem m_drive;
  // Robot's current position and angle
   double robotX;  // Example value
   double robotY;  // Example value
   double robotAngle; // Example value (in degrees)
   double targetX;
   final double targetY = 5.55; 

  /** Creates a new AutoRotateCommand. 
   * @param m_drive */
  public AutoRotateCommand (DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = driveSubsystem;
    m_drive.resetOdometry(new Pose2d(robotX,robotY, new Rotation2d()));
    addRequirements(m_drive);
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
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    robotX = m_drive.getPose().getX();
    robotY = m_drive.getPose().getY();
    robotAngle = m_drive.getPose().getRotation().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (onRedSide()) {
      targetX = 16.5;
    } else {
      targetX = 0.0;
    }

    // Calculate the angle to turn to
    double targetAngle = calculateAngle(robotX, robotY, targetX, targetY);

    // Calculate the angle angle
    double angleDifference = calculateAngleDifference(robotAngle, targetAngle);
   m_drive.resetOdometry(new Pose2d(robotX,robotY, new Rotation2d(targetAngle)));
    System.out.println("Target angle: " + Math.toDegrees(targetAngle) + " degrees");
    System.out.println("Angle difrence: " + Math.toDegrees(angleDifference) + " degrees");
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
