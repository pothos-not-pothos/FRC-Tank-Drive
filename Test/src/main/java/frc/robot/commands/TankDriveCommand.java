// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TankDriveCommand extends Command {
  private final RomiDrivetrain drivetrain;
  private final Supplier<Double> leftSupplier;
  private final Supplier<Double> rightSupplier;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDriveCommand(RomiDrivetrain subsystem, Supplier<Double> leftSupplier, Supplier<Double> rightSupplier) {
    drivetrain = subsystem;
    this.leftSupplier = leftSupplier;
    this.rightSupplier = rightSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public TankDriveCommand(RomiDrivetrain subsystem, double leftSpeed, double rightSpeed){
    this(subsystem, ()-> leftSpeed, ()-> rightSpeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(leftSupplier.get(), rightSupplier.get());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
