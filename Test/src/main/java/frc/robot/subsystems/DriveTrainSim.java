// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSim implements DriveTrainIO {

  private final DifferentialDrivetrainSim simDriveTrain;
  /** Creates a new DriveTrainSim. */
  public DriveTrainSim() {
    simDriveTrain = new DifferentialDrivetrainSim(DCMotor.getFalcon500(2), 10.71, .025, Units.lbsToKilograms(100), Units.inchesToMeters(5), Units.inchesToMeters(24), VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    simDriveTrain.setInputs(leftSpeed, rightSpeed);
  }

  public void stopDrive(){
    simDriveTrain.setInputs(0, 0);
  }

  @Override
  public void periodic() {
    simDriveTrain.update(0.02);
    // This method will be called once per scheduler run
  }

  public void updateInputs(DriveTrainIOInputs inputs){
    inputs.leftPosition = simDriveTrain.getLeftPositionMeters();
    inputs.leftAmp = simDriveTrain.getLeftCurrentDrawAmps();
    inputs.leftVelocity = simDriveTrain.getLeftVelocityMetersPerSecond();
    inputs.rightPosition = simDriveTrain.getRightPositionMeters();
    inputs.rightAmp = simDriveTrain.getRightCurrentDrawAmps();
    inputs.rightVelocity = simDriveTrain.getRightVelocityMetersPerSecond();
  }
  
}
