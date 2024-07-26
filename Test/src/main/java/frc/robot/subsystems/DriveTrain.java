// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  public DriveTrainIO IO;
  public DriveTrainIOInputsAutoLogged inputs = new DriveTrainIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry= new DifferentialDriveOdometry(new Rotation2d(0),0, 0);
  private final DifferentialDriveKinematics kinematics= new DifferentialDriveKinematics(0.141);
  private final Field2d field = new Field2d();
  private double gyroAngle = 0;

  /** Creates a new DriveTrain. */
  public DriveTrain(DriveTrainIO IO) {
    this.IO = IO;
    AutoBuilder.configureRamsete(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // Current ChassisSpeeds supplier
            this::drive, // Method that will drive the robot given ChassisSpeeds
            new ReplanningConfig(), // Default path replanning config. See the API for the options here
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this); // Reference to this subsystem to set requirements
  }

  public double getLeftDistanceInch() {
    return inputs.leftPosition;
  }

  public double getRightDistanceInch() {
    return inputs.rightPosition;
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
   }
   
   public void resetPose(Pose2d newPose2d){
    odometry.resetPosition(new Rotation2d(0),0, 0, newPose2d);
   }
  
   public ChassisSpeeds getChassisSpeeds(){
    double leftSpeed=inputs.leftVelocity;
    double rightSpeed=inputs.rightVelocity;
  
    DifferentialDriveWheelSpeeds wheelSpeeds= new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
    return kinematics.toChassisSpeeds(wheelSpeeds);
   }
  
   public void drive(ChassisSpeeds chassisSpeeds){
  
     DifferentialDriveWheelSpeeds wheelSpeeds= kinematics.toWheelSpeeds(chassisSpeeds);
  
     double leftSpeed=wheelSpeeds.leftMetersPerSecond;
     double rightSpeed=wheelSpeeds.rightMetersPerSecond;
  
    IO.tankDrive(leftSpeed, rightSpeed);
   }

   public void tankDrive(double LeftSpeed, double RightSpeed){
    IO.tankDrive(LeftSpeed, RightSpeed);
   }

   public void stopDrive(){
    IO.stopDrive();
   }


  @Override
  public void periodic() {
   IO.periodic();// This method will be called once per scheduler run
   IO.updateInputs(inputs);
   field.setRobotPose(getPose());
   Twist2d angle = kinematics.toTwist2d(getLeftDistanceInch()/12, getRightDistanceInch()/12);
   gyroAngle=gyroAngle+angle.dtheta;
   odometry.update(new Rotation2d(gyroAngle), getLeftDistanceInch()/12, getRightDistanceInch()/12);
   SmartDashboard.putData(field);

  }

  

}
