// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public final static class ShooterConstants{
    public static final int FLYWHEEL_MAIN_TALON = 6;
    public static final int FLYWHEEL_TOP_TALON = 7;
  }

  public final static class DrivebaseConstants {
    public static final int LEFT_LEADER = 2;
    public static final int LEFT_FOLLOWER = 3;
    public static final int RIGHT_LEADER = 4;
    public static final int RIGHT_FOLLOWER = 5;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
    public static final int LEFT_DRIVE_GROUP_CURRENT_LIMIT = 60;
    public static final int RIGHT_DRIVE_GROUP_CURRENT_LIMIT = 60;

    public static final double GEARING = 10.0;
    public static final double kTrackwidthMeters = 0.63;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double ksVolts = 0.146;
    public static final double kvVoltSecondsPerMeter = 1.26;
    public static final double kaVoltSecondsSquaredPerMeter = 0.165;

    public static final double kPDriveVel = 3;
  }
  
  public final static class IntakeConstants {
    public static final int LEFT_MOTOR = 12; 
    public static final int RIGHT_MOTOR = 13; 
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
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public final static class IndexerConstants {
    public static final int INDEXER_LOWER = 10; //PLACEHOLDER
    public static final int INDEXER_UPPER = 9; //PLACEHOLDER
    public static final double kP = 0; //PLACEHOLDER
    public static final double kI = 0; //PLACEHOLDER
    public static final double kD = 0; //PLACEHOLDER
  }
}