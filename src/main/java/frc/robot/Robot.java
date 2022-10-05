// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.ByteLogReplay;
import org.littletonrobotics.junction.io.LogSocketServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDColors;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private PowerDistribution PD = new PowerDistribution();
  private LED led = new LED();
  private Alliance alliance;

  private RobotContainer robotContainer;
  private Intake intake;
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

    setUseTiming(isReal()); // Run as fast as possible during replay
    LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda1/")); // Log to USB stick (name will be selected automatically)
        Logger.getInstance().addDataReceiver(new LogSocketServer(5800)); // Provide log data over the network, viewable in Advantage Scope.
    } else {
        String path = ByteLogReplay.promptForPath(); // Prompt the user for a file path on the command line
        Logger.getInstance().setReplaySource(new ByteLogReplay(path)); // Read log file for replay
        Logger.getInstance().addDataReceiver(new ByteLogReceiver(ByteLogReceiver.addPathSuffix(path, "_sim"))); // Save replay results to a new log with the "_sim" suffix
    }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    
    robotContainer = new RobotContainer();
    intake = robotContainer.superstructure.intake;
    indexer = robotContainer.superstructure.indexer;
    shooter = robotContainer.superstructure.shooter;
    climber = robotContainer.climber;
    vision = robotContainer.vision;

    alliance = DriverStation.getAlliance();

    CameraServer.startAutomaticCapture();

    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Current", PD.getTotalCurrent());
    SmartDashboard.putNumber("Voltage", PD.getVoltage());

    if (climber.isActive()) {
      led.wave(LEDColors.CLIMBING, 0.05);
    } else if (shooter.isShooting()) {
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
    } else if (isEnabled()) {
      led.wave(alliance == Alliance.Red ? LEDColors.RED : LEDColors.BLUE, 0.05);
    } else {
      led.setColor(alliance == Alliance.Red ? LEDColors.RED : LEDColors.BLUE);
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

    if (autonomousCommand != null) autonomousCommand.cancel();

    alliance = DriverStation.getAlliance();
  }

  @Override
  public void teleopPeriodic() {
    if (getSecondsRemaining() < 20 && getSecondsRemaining() > 18) {
      robotContainer.setDriverRumble(1);
    } else if (shooter.isShooting() && !vision.hasTarget()) {
      robotContainer.setDriverRumble(0.25);
    } else {
      robotContainer.setDriverRumble(0);
    }

    if (intake.hasBall()) {
      robotContainer.setOperatorRumble(0.5);
    } else {
      robotContainer.setOperatorRumble(0);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}