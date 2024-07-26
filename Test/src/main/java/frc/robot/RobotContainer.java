// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainSim;
import frc.robot.subsystems.DriveTrainSparkMax;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_romiDrivetrain;
  private final Joystick joystick = new Joystick(0);
  private final Shooter shooter = new Shooter(ShooterConstants.upperID, ShooterConstants.lowerID);
  private final Intake intake = new Intake(IntakeConstants.motorID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Robot.isReal()){
     DriveTrainSparkMax real = new DriveTrainSparkMax();
     m_romiDrivetrain = new DriveTrain(real);
    }
    else {
      DriveTrainSim notReal = new DriveTrainSim();
      m_romiDrivetrain = new DriveTrain(notReal);
    }
    

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(new TankDriveCommand(m_romiDrivetrain, () -> joystick.getRawAxis(0), () -> joystick.getRawAxis(5)));
    intake.setDefaultCommand(new SpinIntakeCommand(intake, IntakeConstants.intakeSpeed));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PathPlannerAuto("Example Path");
  }

  

}
