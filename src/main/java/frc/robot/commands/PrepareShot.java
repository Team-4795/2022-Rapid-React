// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

public class PrepareShot extends CommandBase {
  private final Drivebase drivebase;
  private final Superstructure superstructure;
  private final Vision vision;
  private boolean isAligned;
  private long alignStart;

  public PrepareShot(Drivebase drivebase, Superstructure superstructure, Vision vision) {
    this.drivebase = drivebase;
    this.superstructure = superstructure;
    this.vision = vision;

    addRequirements(drivebase, superstructure);
  }

  @Override
  public void initialize() {
    isAligned = false;
    alignStart = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    isAligned = true;

    var robotPose = drivebase.getPose();
    var goalPose = drivebase.getGoalPose();
    double rotation = Math.toDegrees(Math.atan2(robotPose.getY() - goalPose.getY(), robotPose.getX() - goalPose.getX()));
    double goalAngle = (robotPose.getRotation().getDegrees() - rotation) % 360;
    double goalDistance = Units.metersToFeet(drivebase.getGoalPose().getTranslation().getDistance(drivebase.getPose().getTranslation()));

    double verticalSpeed = 0;
    double angularSpeed = 0;

    if (Math.abs(goalAngle) < 45 && (goalDistance < 6 || goalDistance > 13)) {
      isAligned = false;
      verticalSpeed = Math.signum(9 - goalDistance);

      superstructure.shooter.setShooterPower(0, 0);
    } else {
      var preset = Shoot.interpolate(goalDistance - 3.5);

      superstructure.shooter.setShooterRPM(preset.mainRPM * 0.6, preset.topRPM * 0.6);
    }

    if (Math.abs(goalAngle) > 4) {
      if (!vision.hasTarget() || Math.abs(goalAngle) > 15) isAligned = false;

      angularSpeed = MathUtil.clamp(Math.abs(goalAngle / 90), 0.15, 0.6) * Math.signum(goalAngle);
      verticalSpeed *= 1 - Math.sqrt(angularSpeed);
    }

    drivebase.arcadeDrive(verticalSpeed, angularSpeed);

    if (isAligned && System.currentTimeMillis() - alignStart > 250) alignStart = System.currentTimeMillis();

    if (superstructure.intake.isExtended()) {
      if (superstructure.indexer.hasUpperBall() && superstructure.indexer.hasLowerBall()) superstructure.intake.retract();

      superstructure.intake.setSpeed(0.75);
    } else {
      superstructure.intake.setSpeed(0);
    }

    superstructure.indexer.setIndexerSpeed(0, 0);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return isAligned && System.currentTimeMillis() - alignStart > 200;
  }
}