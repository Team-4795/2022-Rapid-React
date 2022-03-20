// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
  public static final class DrivebaseConstants {
    public static final int LEFT_LEADER = 2;
    public static final int LEFT_FOLLOWER = 3;
    public static final int RIGHT_LEADER = 4;
    public static final int RIGHT_FOLLOWER = 5;

    public static final int LEFT_DRIVE_GROUP_CURRENT_LIMIT = 60;
    public static final int RIGHT_DRIVE_GROUP_CURRENT_LIMIT = 60;

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

  public static final class ClimberConstants {
    public static final int CLIMB_MOTOR = 11;
  }
 
  public static final class VisionConstants {
    public static final String CAMERA_NAME = "LifeCam";

    public static final double CAMERA_OFFSET_METERS = Units.inchesToMeters(12);
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(25.25);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(104);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(42.5);
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

  public static final class RGBPreset {
    public int r;
    public int g;
    public int b;
    public double percent;

    public RGBPreset(int r, int g, int b, double ... percent) {
      this.r = r;
      this.g = g;
      this.b = b;

      if (percent.length == 1) {
        this.percent = MathUtil.clamp(percent[0], 0, 1);
      } else {
        this.percent = 1;
      }
    }
  }

  public static final class HSVPreset {
    public int h;
    public int s;
    public int v;
    public double percent;

    public HSVPreset(int h, int s, int v, double ... percent) {
      this.h = h;
      this.s = s;
      this.v = v;

      if (percent.length == 1) {
        this.percent = MathUtil.clamp(percent[0], 0, 1);
      } else {
        this.percent = 1;
      }
    }
  }

  public static final class LEDColors {
    public static final HSVPreset RED = new HSVPreset(0, 255, 255);
    public static final HSVPreset BLUE = new HSVPreset(125, 255, 255);
    public static final RGBPreset ONE_BALL = new RGBPreset(0, 128, 0, 0.5);
    public static final RGBPreset TWO_BALLS = new RGBPreset(0, 128, 0);
    public static final RGBPreset SHOOTER_INIT = new RGBPreset(0, 128, 128); //cyan
    public static final RGBPreset TARGET_FOUND = new RGBPreset(128, 108, 0); //orange
    public static final RGBPreset SHOOTER_CHARGING = new RGBPreset(0, 128, 0); //green
    public static final RGBPreset SHOOTING = new RGBPreset(235, 52, 119); // hot pink
    public static final RGBPreset IDLE = new RGBPreset(128, 0, 128); //magenta
    public static final RGBPreset CLIMBER_DOWN = new RGBPreset(64, 128, 0); //light green CHANGE LATER 
    public static final RGBPreset CLIMBER_DOWN2 = new RGBPreset(64, 0, 128); //violet
    public static final RGBPreset CLIMBER_UP = new RGBPreset(128, 108, 0); //Gold-ish
    public static final RGBPreset CLIMBER_UP2 = new RGBPreset(0, 20, 128); //light green CHANGE LATER 
  }
}