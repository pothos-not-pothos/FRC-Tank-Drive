// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public interface DriveTrainIO {
    public class DriveTrainIOInputs{
        public double leftPosition = 0;
        public double leftVoltage = 0;
        public double leftAmp = 0;
        public double leftVelocity = 0;
        public double rightPosition = 0;
        public double rightVoltage = 0;
        public double rightAmp = 0;
        public double rightVelocity = 0;

    }

    public abstract void tankDrive(double leftSpeed, double rightSpeed);
    public abstract void periodic();
    public abstract void updateInputs(DriveTrainIOInputs inputs);

}