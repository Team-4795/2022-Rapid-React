// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Solenoid;

public class PneumaticsCommand extends CommandBase {
  private final Solenoid solenoid;
  /** Create new pneumatics commands. */
  public PneumaticsCommand(Solenoid solenoid) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.solenoid = solenoid;
    addRequirements(solenoid);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    solenoid.forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    solenoid.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
