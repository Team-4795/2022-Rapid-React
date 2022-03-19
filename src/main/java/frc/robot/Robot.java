// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private PowerDistribution PD = new PowerDistribution(1, ModuleType.kRev);
  private LED led = new LED();
  private Alliance alliance;

  private RobotContainer robotContainer;

  private long teleopStart;

  private double getSecondsRemaining() {
    return 135.0 - (System.currentTimeMillis() - teleopStart) / 1000.0;
  }

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    alliance = DriverStation.getAlliance();

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Current", PD.getTotalCurrent());
    SmartDashboard.putNumber("Voltage", PD.getVoltage());

    if (robotContainer.superstructure.shooter.getTargetRPM() > 0) {
      if (robotContainer.superstructure.indexer.isActive()) {
        led.setColor(150, 0, 150, 1);
      } else {
        double percent = robotContainer.superstructure.shooter.getTargetRPM() - robotContainer.superstructure.shooter.getMainRPM();
        percent = 1.0 - Math.abs(percent / robotContainer.superstructure.shooter.getTargetRPM());
  
        led.setColor(200, 90, 240, percent);
      }
    } else if (robotContainer.superstructure.indexer.hasUpperBall()) {
      if (robotContainer.superstructure.indexer.hasLowerBall()) {
        led.setColor(0, 128, 0, 1);
      } else {
        led.setColor(0, 128, 0, 0.5);
      }
    } else if (alliance == Alliance.Red) {
      led.setColor(128, 0, 0, 1);
    } else {
      led.setColor(0, 0, 128, 1);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    alliance = DriverStation.getAlliance();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    teleopStart = System.currentTimeMillis();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    alliance = DriverStation.getAlliance();
  }

  @Override
  public void teleopPeriodic() {
    if (getSecondsRemaining() < 20 && getSecondsRemaining() > 18) {
      robotContainer.setRumble(1);
    } else {
      robotContainer.setRumble(0);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}