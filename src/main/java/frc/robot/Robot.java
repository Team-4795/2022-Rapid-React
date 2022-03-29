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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDColors;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private PowerDistribution PD = new PowerDistribution(1, ModuleType.kRev);
  private LED led = new LED();
  private Alliance alliance;

  private RobotContainer robotContainer;
  private Indexer indexer;
  private Shooter shooter;
  private Climber climber;
  private Vision vision;

  private long teleopStart;

  private double getSecondsRemaining() {
    return 135.0 - (System.currentTimeMillis() - teleopStart) / 1000.0;
  }

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    indexer = robotContainer.superstructure.indexer;
    shooter = robotContainer.superstructure.shooter;
    climber = robotContainer.climber;
    vision = robotContainer.vision;

    alliance = DriverStation.getAlliance();

    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Current", PD.getTotalCurrent());
    SmartDashboard.putNumber("Voltage", PD.getVoltage());

    if (climber.isActive()) {
      led.wave(LEDColors.CLIMBING, 0.05);
    } else if (shooter.getTargetRPM() > 0) {
      if (indexer.isActive()) {
        led.setColor(LEDColors.SHOOTING);
      } else {
        double percent = shooter.getTargetRPM() - shooter.getMainRPM();
        percent = 1.0 - Math.abs(percent / shooter.getTargetRPM());

        led.setColor(LEDColors.SHOOTER_CHARGING, percent);
      }
    } else if (robotContainer.superstructure.indexer.hasUpperBall()) {
      if (robotContainer.superstructure.indexer.hasLowerBall()) {
        led.setColor(LEDColors.HAS_BALL, 1);
      } else {
        led.setColor(LEDColors.HAS_BALL, 0.5);
      }
    } else if (alliance == Alliance.Red) {
      led.wave(LEDColors.RED, 0.05);
    } else {
      led.wave(LEDColors.BLUE, 0.05);
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
    } else if (shooter.getTargetRPM() > 0 && shooter.getMainRPM() > 650 && !vision.hasTarget()) {
      robotContainer.setRumble(0.25);
    } else {
      robotContainer.setRumble(0);
    }
  }

  @Override
  public void testInit() {
    LiveWindow.disableAllTelemetry();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}