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
    public static final double WHEEEEEEEEEEEEEEEEEEEL_DIAMETER_METERS = Units.inchesToMeters(6);
    public static final int LEFT_DRIVE_GROUP_CURRENT_LIMIT = 60;
    public static final int RIGHT_DRIVE_GROUP_CURRENT_LIMIT = 60;

    public static final double GEARING = 10.0;
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
    public static final int ROTATION_JOYSTICK = 4;
    public static final int THROTTLE_TRIGGER = 4;
    public static final int ROTATE_IN_PLACE_BUTTON = 6;
    public static final int Xbutton = 40;
    public static final int Ybutton = 60;
  }

  public final static class ClimberConstants {
    public static final int climb_motor = 12;
    public static final double spool_diameter = 1.375;
    public static final double stage_length = 27.5;
    public static final double safety_margin = 2;
    public static final int hall_effect_port = 0;
    public static final double physical_gear = 46/16;
    public static final double versaplanetary = 7;
    public static final double starting_hook_height = 35.125;
    public static final double bar_height = 60.25;
    

  }
}