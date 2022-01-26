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

  private AHRS gyro;

  private DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

  private final edu.wpi.first.math.kinematics.DifferentialDriveOdometry odometry;

  public Drivebase() {
    leftLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();

    rightLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);

    rightLeader.setInverted(false);
    rightFollower.setInverted(false);

    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();

    resetEncoders();

    leftLeader.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightLeader.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);

    leftFollower.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightFollower.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);

    gyro = new AHRS(SPI.Port.kMXP);

    odometry = new edu.wpi.first.math.kinematics.DifferentialDriveOdometry(gyro.getRotation2d());

    diffDrive.setDeadband(0.02);
  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    diffDrive.curvatureDrive(speed, rotation, quickTurn);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightFollower.setVoltage(rightVolts);
    diffDrive.feed();
  }
  //ENCODER STUFF
  public double getLeftWheelEncoder() {
    return m_leftEncoder.getPosition();
  }

  public double getRightWheelEncoder() {
    return m_rightEncoder.getPosition();
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
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
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    double leftDistance = getLeftWheelEncoder() / DrivebaseConstants.GEARING * DrivebaseConstants.WHEEEEEEEEEEEEEEEEEEEL_DIAMETER_METERS * Math.PI;
    double rightDistance = getRightWheelEncoder() / DrivebaseConstants.GEARING * DrivebaseConstants.WHEEEEEEEEEEEEEEEEEEEL_DIAMETER_METERS * Math.PI;

    odometry.update(gyro.getRotation2d(), leftDistance, rightDistance);

    SmartDashboard.putNumber("Left distance", leftDistance);
    SmartDashboard.putNumber("Right distance", rightDistance);
  }
}
