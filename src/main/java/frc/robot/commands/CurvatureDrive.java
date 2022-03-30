// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;

import frc.robot.subsystems.Drivebase;

public class CurvatureDrive extends CommandBase {
  private final Drivebase drivebase;
  private final Supplier<Double> speedSupplier;
  private final Supplier<Double> rotationSupplier;
  private final Supplier<Double> throttleSupplier;
  private long lastAcceleration;
  private long lastUpdate;
  private double speed = 0;
  private double minSpeed = 0.05;

  public CurvatureDrive(
      Drivebase drivetrain,
      Supplier<Double> SpeedSupplier,
      Supplier<Double> RotationSupplier,
      Supplier<Double> throttle) {
    drivebase = drivetrain;
    speedSupplier = SpeedSupplier;
    rotationSupplier = RotationSupplier;
    throttleSupplier = throttle;
    
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivebase.disableBrakeMode();
    lastAcceleration = 0;
    lastUpdate = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    double acceleration = speedSupplier.get();
    double rotation = rotationSupplier.get();
    double throttle = throttleSupplier.get();

    acceleration = 0.5 * acceleration + 0.5 * Math.pow(acceleration, 3);
    rotation = 0.5 * rotation + 0.5 * Math.pow(rotation, 3);
    throttle = (1.0 - throttle * 0.65);
    
    if(Math.abs(acceleration) > 0) {
      lastAcceleration = System.currentTimeMillis();

      if(Math.signum(speed) != Math.signum(acceleration)) {
        speed = Math.copySign(minSpeed, acceleration);
      } else {
        double adjustment = (System.currentTimeMillis() - lastUpdate) / 500.0;
        
        speed = Math.min(Math.abs(speed) + adjustment, Math.abs(acceleration));
        speed = Math.copySign(speed * throttle * (1.0 - minSpeed) + minSpeed, acceleration);
      }
    } else {
      speed = 0;
    }
    
    if(speed == 0) {
      double transitionRamp = MathUtil.clamp((System.currentTimeMillis() - lastAcceleration) / 250.0, 0.5, 1.0);

      rotation *= Math.max(throttle, 0.4) * transitionRamp * 0.5;

      if(Math.abs(rotation) > 0) rotation = Math.copySign(Math.abs(rotation) * (1.0 - minSpeed) + minSpeed, rotation);

      drivebase.curvatureDrive(speed, rotation, true);
    } else {
      drivebase.curvatureDrive(speed, rotation, false);
    }

    lastUpdate = System.currentTimeMillis();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}