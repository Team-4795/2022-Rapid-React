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
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {
  private final CANSparkMax leftLeader = new CANSparkMax(DrivebaseConstants.LEFT_LEADER, MotorType.kBrushless);
  private final CANSparkMax leftFollower = new CANSparkMax(DrivebaseConstants.LEFT_FOLLOWER, MotorType.kBrushless);

  private final CANSparkMax rightLeader = new CANSparkMax(DrivebaseConstants.RIGHT_LEADER, MotorType.kBrushless);
  private final CANSparkMax rightFollower = new CANSparkMax(DrivebaseConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  private final RelativeEncoder m_rightEncoder;
  private final RelativeEncoder m_leftEncoder;

  private final AHRS gyro;

  private final DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

  private final DifferentialDriveOdometry odometry;

  private final Field2d m_field2d = new Field2d();

  private double movementSpeed = 0;
  private double direction = 1;

  public Drivebase() {
    leftLeader.restoreFactoryDefaults();
    leftFollower.restoreFactoryDefaults();
    rightLeader.restoreFactoryDefaults();
    rightFollower.restoreFactoryDefaults();

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kCoast);
    leftFollower.setIdleMode(IdleMode.kCoast);
    rightLeader.setIdleMode(IdleMode.kCoast);
    rightFollower.setIdleMode(IdleMode.kCoast);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);
    rightLeader.setInverted(false);
    rightFollower.setInverted(false);

    m_leftEncoder = leftLeader.getEncoder();
    m_rightEncoder = rightLeader.getEncoder();

    // velocity calculation, gearing * wheel diameter, meters per second
    m_leftEncoder.setVelocityConversionFactor((1.0 / 60.0 / DrivebaseConstants.GEARING) * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI);
    m_rightEncoder.setVelocityConversionFactor((1.0 / 60.0 / DrivebaseConstants.GEARING) * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI);

    resetEncoders();

    leftLeader.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightLeader.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);

    leftFollower.setSmartCurrentLimit(DrivebaseConstants.LEFT_DRIVE_GROUP_CURRENT_LIMIT);
    rightFollower.setSmartCurrentLimit(DrivebaseConstants.RIGHT_DRIVE_GROUP_CURRENT_LIMIT);

    gyro = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    SmartDashboard.putData(m_field2d);
  }

  public void enableBrakeMode() {
    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);
  }

  public void disableBrakeMode() {
    leftLeader.setIdleMode(IdleMode.kCoast);
    leftFollower.setIdleMode(IdleMode.kCoast);
    rightLeader.setIdleMode(IdleMode.kCoast);
    rightFollower.setIdleMode(IdleMode.kCoast);
  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    movementSpeed = Math.max(Math.abs(speed), Math.abs(rotation));

    diffDrive.curvatureDrive(speed * direction, rotation, quickTurn);
  }

  public void setDirection(double d) {
    direction = d;
  }

  public double getDirection() {
    return direction;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    diffDrive.feed();
  }

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

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  public void reverse() {
    if(Math.abs(movementSpeed) < 0.5) direction *= -1;
  }

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

  public void putTrajectory(Trajectory trajectory) {
    m_field2d.getObject("traj").setTrajectory(trajectory);
  }

  @Override
  public void periodic() {
    double leftDistance = getLeftWheelEncoder() / DrivebaseConstants.GEARING * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI;
    double rightDistance = getRightWheelEncoder() / DrivebaseConstants.GEARING * DrivebaseConstants.WHEEL_DIAMETER_METERS * Math.PI;

    odometry.update(gyro.getRotation2d(), leftDistance, rightDistance);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Drivebase");
    builder.addDoubleProperty("Left speed", m_leftEncoder::getVelocity, null);
    builder.addDoubleProperty("Right speed", m_rightEncoder::getVelocity, null);
    builder.addDoubleProperty("Gyro angle", gyro.getRotation2d()::getDegrees, null);
  }
}