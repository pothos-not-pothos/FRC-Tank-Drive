// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

private final CANSparkMax uppershootermotor;
private final CANSparkMax lowershootermotor;

  public Shooter(int upperID, int lowerID) {
    this.uppershootermotor=new CANSparkMax(upperID, MotorType.kBrushless);
      uppershootermotor.setIdleMode(IdleMode.kBrake);
      uppershootermotor.setInverted(false);
      uppershootermotor.setSmartCurrentLimit(40);

    this.lowershootermotor=new CANSparkMax(lowerID, MotorType.kBrushless);
      lowershootermotor.setIdleMode(IdleMode.kBrake);
      lowershootermotor.setInverted(true);
      lowershootermotor.setSmartCurrentLimit(40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinShooter(double upperSpeed, double lowerSpeed){
    uppershootermotor.set(upperSpeed);
    lowershootermotor.set(lowerSpeed);
  }

  public void stopShooter(){
    uppershootermotor.stopMotor();
    lowershootermotor.stopMotor();
  }
}
