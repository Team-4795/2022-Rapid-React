// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);

  private boolean hasTarget = false;
  private double targetDistance = 0;
  private double targetAngle = 0;

  public Vision() {
    camera.setDriverMode(false);
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getTargetDistance() {
    return targetDistance;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public void enableLED() {
    camera.setLED(VisionLEDMode.kOn);
  }

  public void disableLED() {
    camera.setLED(VisionLEDMode.kOff);
  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      hasTarget = true;
      targetDistance = Units.metersToFeet(PhotonUtils.calculateDistanceToTargetMeters(
        VisionConstants.CAMERA_HEIGHT_METERS,
        VisionConstants.TARGET_HEIGHT_METERS,
        VisionConstants.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(result.getBestTarget().getPitch())) - VisionConstants.CAMERA_OFFSET_METERS
        );
      targetDistance -= Math.pow(targetDistance * 0.1, 2);
      targetAngle = result.getBestTarget().getYaw();
    } else {
      hasTarget = false;
      targetDistance = -1;
      targetAngle = -1;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Vision");
    builder.addBooleanProperty("Has target", () -> hasTarget, null);
    builder.addDoubleProperty("Goal distance", () -> targetDistance, null);
    builder.addDoubleProperty("Goal angle", () -> targetAngle, null);
  }
}