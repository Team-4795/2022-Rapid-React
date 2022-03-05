// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static final class DrivebaseConstants {
    public static final int LEFT_LEADER = 2;
    public static final int LEFT_FOLLOWER = 3;
    public static final int RIGHT_LEADER = 4;
    public static final int RIGHT_FOLLOWER = 5;

    public static final int LEFT_DRIVE_GROUP_CURRENT_LIMIT = 60;
    public static final int RIGHT_DRIVE_GROUP_CURRENT_LIMIT = 60;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final double GEARING = 10.0;
    public static final double kTrackwidthMeters = 0.63;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double ksVolts = 0.146;
    public static final double kvVoltSecondsPerMeter = 1.26;
    public static final double kaVoltSecondsSquaredPerMeter = 0.165;

    public static final double kPDriveVel = 3;
  }
  
  public static final class IntakeConstants {
    public static final int LEFT_MOTOR = 12;
    public static final int RIGHT_MOTOR = 13;

    public static final int FORWARD_CHANNEL = 0;
    public static final int REVERSE_CHANNEL = 1;
  }
  
  public static final class IndexerConstants {
    public static final int INDEXER_LOWER = 10;
    public static final int INDEXER_UPPER = 9;
    public static final int BREAK_BEAM_PORT = 1;

    public static final int CURRENT_LIMIT = 30;
  }

  public static final class ShooterConstants {
    public static final int FLYWHEEL_MAIN_TALON = 6;
    public static final int FLYWHEEL_TOP_TALON = 7;
  }
 
  public static final class VisionConstants {
    public static final String CAMERA_NAME = "mmal_service_16.1";

    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(27.5);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(103.5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(35);
  }

  public static final class ControllerConstants {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;

    public static final double JOYSTICK_DEADBAND = 0.05;
  }

  public static final class AutoConstants {
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class Preset {
    public double distance;
    public double topRPM;
    public double mainRPM;

    public Preset(double t, double m, double d) {
      distance = d;
      topRPM = t;
      mainRPM = m;
    }
  }
}