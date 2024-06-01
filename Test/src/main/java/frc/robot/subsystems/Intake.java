// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final CANSparkMax intakeMotor;
  /** Creates a new Intake. */
  public Intake(int motorID) {
    this.intakeMotor=new CANSparkMax(motorID, MotorType.kBrushless);
      intakeMotor.setIdleMode(IdleMode.kBrake);
      intakeMotor.setInverted(false);
      intakeMotor.setSmartCurrentLimit(40);
  }

  @Override
  public void periodic() {
   // This method will be called once per scheduler run
  }

  public void spinIntake(double intakeSpeed){
    intakeMotor.set(intakeSpeed);
  }

  public void stopIntake(){
    intakeMotor.stopMotor();
  }
}