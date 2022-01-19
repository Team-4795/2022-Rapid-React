// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {

  private CANSparkMax leftLeader = new CANSparkMax(DrivebaseConstants.LEFT_LEADER, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(DrivebaseConstants.LEFT_FOLLOWER, MotorType.kBrushless);

  private CANSparkMax rightLeader = new CANSparkMax(DrivebaseConstants.RIGHT_LEADER, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(DrivebaseConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  // The left-side drive encoder
  Encoder leftEncoders = new Encoder(DrivebaseConstants.LEFT_LEADER, DrivebaseConstants.LEFT_FOLLOWER, false, Encoder.EncodingType.k2X);

  // The right-side drive encoder
  Encoder rightEncoders = new Encoder(DrivebaseConstants.RIGHT_LEADER, DrivebaseConstants.RIGHT_FOLLOWER, false, Encoder.EncodingType.k2X);

  private double leftEncoderStart, rightEncoderStart;

  private AHRS gyro;

  private DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

  public Drivebase() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    leftLeader.setInverted(true);
    leftFollower.setInverted(true);

    // Configures the encoder to return a distance of 4 for every 256 pulses
    // Also changes the units of getRate  
    leftEncoders.setDistancePerPulse(4./256.);
    rightEncoders.setDistancePerPulse(4./256.);

    gyro = new AHRS(SPI.Port.kMXP);

    leftEncoderStart = leftEncoders.getDistance();
    rightEncoderStart = rightEncoders.getDistance();

    diffDrive.setDeadband(0.02);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void setMaxOutput(double maxOutput) {
    diffDrive.setMaxOutput(maxOutput);
}

  //ENCODER STUFF
  double getRightWheelDistance = rightEncoders.getDistance();
  double getLeftWheelDistance = leftEncoders.getDistance();

  public void resetEncoders() {
    rightEncoders.reset();
    leftEncoders.reset();
  }

  public double getAverageEncoderDistance() {
    return (leftEncoders.getDistance() + rightEncoders.getDistance()) / 2.0;
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

  @Override
  public void periodic() {}
}
