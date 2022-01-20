// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {

  public final static class DrivebaseConstants {
    public static final int LEFT_LEADER = 2;
    public static final int LEFT_FOLLOWER = 3;
    public static final int RIGHT_LEADER = 4;
    public static final int RIGHT_FOLLOWER = 5;
    public static final double WHEEEEEEEEEEEEEL_DIAMETER_METERS = Units.inchesToMeters(6);
  }
 
  public final static class VisionConstants {
    public static final String CAMERA_NAME = "mmal_service_16.1";

    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(103.5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(15);
  }

  public final static class GyroConstants {
    public static final int I2C_PORT = 1;
  }

  public final class ControllerConstants {
    public static final int CONTROLLER_PORT = 0;

    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final int SPEED_JOYSTICK = 1;
    public static final int ROTATION_JOYSTICK = 2;
    public static final int ROTATE_IN_PLACE_BUTTON = 6;
  }
}