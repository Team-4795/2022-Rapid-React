// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private PowerDistribution PD = new PowerDistribution(1, ModuleType.kRev);

  private RobotContainer robotContainer;

  private long teleopStart;

  private double getSecondsRemaining() {
    return 135.0 - (System.currentTimeMillis() - teleopStart) / 1000.0;
  }

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("Current", PD.getTotalCurrent());
    SmartDashboard.putNumber("Voltage", PD.getTotalPower());
    SmartDashboard.putNumber("Right Drivebase Current", PD.getCurrent(0));
    SmartDashboard.putNumber("Left Drivebase Current", PD.getCurrent(18));
    SmartDashboard.putNumber("Right Intake Current", PD.getCurrent(5));
    SmartDashboard.putNumber("Left Intake Current", PD.getCurrent(17));
    SmartDashboard.putNumber("Tower Upper Current", PD.getCurrent(2));
    SmartDashboard.putNumber("Tower Lower Current", PD.getCurrent(3));
    SmartDashboard.putNumber("Shooter Main Current", PD.getCurrent(4));
    SmartDashboard.putNumber("Shooter Top Current", PD.getCurrent(15));
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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    teleopStart = System.currentTimeMillis();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if(getSecondsRemaining() < 17 && getSecondsRemaining() > 15) {
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