// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Shoot extends CommandBase {
  private final Drivebase drivebase;
  private final Shooter shooter;
  private final Vision vision;

  public Shoot(Drivebase drivebase, Shooter shooter, Vision vision) {
    this.drivebase = drivebase;
    this.shooter = shooter;
    this.vision = vision;

    addRequirements(drivebase, shooter);
  }

  @Override
  public void execute() {
    if(!vision.hasTarget()) return;

    double angle = vision.getTargetAngle();
    double turnSpeed = -angle / 400.0;
    
    turnSpeed = MathUtil.clamp(Math.copySign(Math.max(Math.abs(turnSpeed), 0.035), turnSpeed), -0.15, 0.15);

    if(Math.abs(angle) > 2) {
      drivebase.curvatureDrive(0, turnSpeed, true);
    } else {
      drivebase.curvatureDrive(0, 0, false);
    }

    shooter.setShooterRPM(5000, 4000);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
