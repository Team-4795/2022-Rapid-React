// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class curveDrive extends CommandBase {

  private final Drivebase m_drivebaseSubsystem;

  Supplier<Double> speedValue;
  Supplier<Double> rotationValue;
  Supplier<Double> throttleSupplier;
  Supplier<Boolean> quickTurnValue;
  
  /** Creates a new curveDrive. */
  public curveDrive(Drivebase subsystem, Supplier<Double> xaxisSpeedSupplier, Supplier<Double> zaxisRotateSupplier, Supplier<Boolean> quickTurnSupplier, Supplier<Double> m_throttleSupplier) {

    m_drivebaseSubsystem = subsystem;

    speedValue = xaxisSpeedSupplier;
    rotationValue = zaxisRotateSupplier;
    throttleSupplier = m_throttleSupplier;
    quickTurnValue = quickTurnSupplier;
    //do you see this?

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivebaseSubsystem.curvatureDrive(speedValue.get() * (1 - throttleSupplier.get() * 0.3), rotationValue.get() * (1 - throttleSupplier.get() * 0.3), quickTurnValue.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
