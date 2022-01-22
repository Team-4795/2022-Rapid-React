// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {

  private CANSparkMax leftLeader = new CANSparkMax(DrivebaseConstants.LEFT_LEADER, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(DrivebaseConstants.LEFT_FOLLOWER, MotorType.kBrushless);

  private CANSparkMax rightLeader = new CANSparkMax(DrivebaseConstants.RIGHT_LEADER, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(DrivebaseConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  private RelativeEncoder m_rightEncoder;
  private RelativeEncoder m_leftEncoder;

  // The left-side drive encoder
  Encoder leftEncoders = new Encoder(DrivebaseConstants.LEFT_LEADER, DrivebaseConstants.LEFT_FOLLOWER, false, Encoder.EncodingType.k2X);

  // The right-side drive encoder
  Encoder rightEncoders = new Encoder(DrivebaseConstants.RIGHT_LEADER, DrivebaseConstants.RIGHT_FOLLOWER, false, Encoder.EncodingType.k2X);

  private double leftEncoderStart, rightEncoderStart;

  private AHRS gyro;

  private double leftStart;
  
  private double rightStart;

  private DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

  private final edu.wpi.first.math.kinematics.DifferentialDriveOdometry odometry;

  public Drivebase() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);

    rightLeader.setInverted(true);
    rightFollower.setInverted(true);

    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();

    leftStart = m_leftEncoder.getPosition();
    rightStart = m_rightEncoder.getPosition();

    resetEncoders();

    leftLeader.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightLeader.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);

    leftFollower.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightFollower.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);

    // Configures the encoder to return a distance of 4 for every 256 pulses
    // Also changes the units of getRate  
    leftEncoders.setDistancePerPulse(4./256.);
    rightEncoders.setDistancePerPulse(4./256.);

    gyro = new AHRS(SPI.Port.kMXP);

    leftEncoderStart = leftEncoders.getDistance();
    rightEncoderStart = rightEncoders.getDistance();

    odometry = new edu.wpi.first.math.kinematics.DifferentialDriveOdometry(gyro.getRotation2d());

    diffDrive.setDeadband(0.02);
  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    diffDrive.curvatureDrive(speed, rotation, quickTurn);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  //ENCODER STUFF
  public double getLeftWheelEncoder() {
    return m_leftEncoder.getPosition() - leftEncoderStart;
  }

  public double getRightWheelEncoder() {
    return m_rightEncoder.getPosition() - rightEncoderStart;
  }

  public void resetEncoders() {
    leftEncoderStart = m_leftEncoder.getPosition();
    rightEncoderStart = m_rightEncoder.getPosition();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoders.getRate(), rightEncoders.getRate());
  }

  //GYRO STUFF
  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public void reverse() {}

  //ODOMETRY STUFF
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    double leftDistance = getLeftWheelEncoder() / 10.0 * DrivebaseConstants.WHEEEEEEEEEEEEEEEEEEEL_DIAMETER_METERS * Math.PI;
    double rightDistance = getRightWheelEncoder() / 10.0 * DrivebaseConstants.WHEEEEEEEEEEEEEEEEEEEL_DIAMETER_METERS * Math.PI;

    odometry.update(gyro.getRotation2d(), leftDistance, rightDistance);

    SmartDashboard.putNumber("Left distance", leftDistance);
    SmartDashboard.putNumber("Right distance", rightDistance);
  }
}
