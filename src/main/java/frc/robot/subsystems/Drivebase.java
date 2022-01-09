// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DrivebaseConstants;

public class Drivebase extends SubsystemBase {
  private CANSparkMax leftLeader = new CANSparkMax(DrivebaseConstants.LEFT_LEADER, MotorType.kBrushless);
  private CANSparkMax leftFollower = new CANSparkMax(DrivebaseConstants.LEFT_FOLLOWER, MotorType.kBrushless);

  private CANSparkMax rightLeader = new CANSparkMax(DrivebaseConstants.RIGHT_LEADER, MotorType.kBrushless);
  private CANSparkMax rightFollower = new CANSparkMax(DrivebaseConstants.RIGHT_FOLLOWER, MotorType.kBrushless);

  private DifferentialDrive diffDrive = new DifferentialDrive(leftLeader, rightLeader);

  private double movementSpeed = 0;
  private int direction = 1;

  public Drivebase() {
    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    leftLeader.setIdleMode(IdleMode.kBrake);
    leftFollower.setIdleMode(IdleMode.kBrake);
    rightLeader.setIdleMode(IdleMode.kBrake);
    rightFollower.setIdleMode(IdleMode.kBrake);

    diffDrive.setDeadband(0.0);
  }

  public void curvatureDrive(double speed, double rotation, boolean quickTurn) {
    movementSpeed = Math.max(Math.abs(speed), Math.abs(rotation));

    diffDrive.curvatureDrive(speed * direction, rotation, quickTurn);
  }

  public void reverse() {
    if(Math.abs(movementSpeed) < 0.3) direction *= -1;
  }

  @Override
  public void periodic() {}
}
