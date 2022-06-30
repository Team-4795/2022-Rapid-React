// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class IndependentDrive extends CommandBase {
  private final Drivebase drivebase;
  private final Supplier<Double> leftSupplier;
  private final Supplier<Double> rightSupplier;

  public IndependentDrive(
      Drivebase drivetrain,
      Supplier<Double> LeftSupplier,
      Supplier<Double> RightSupplier) {
    drivebase = drivetrain;
    leftSupplier = LeftSupplier;
    rightSupplier = RightSupplier;    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivebase.disableBrakeMode();
  }

  @Override
  public void execute() {
    drivebase.tankDriveVolts(leftSupplier.get(), rightSupplier.get());

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}