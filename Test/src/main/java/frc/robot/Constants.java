// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ShooterConstants {
        public static final int upperID=10;
        public static final int lowerID=11;

        public static final double upperSpeed=1;
        public static final double lowerSpeed=1;
    }

    public static final class DriveConstants {
        public static final double leftSpeed=-1;
        public static final double rightSpeed=-1;
    }
    
    public static final class IntakeConstants {
        public static final int motorID=20;

        public static final double intakeSpeed=1;
    }
    

}
