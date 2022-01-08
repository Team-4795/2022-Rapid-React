// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.EastDrive;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  private final Drivebase drivebase = new Drivebase();
  
  private final XboxController controller = new XboxController(ControllerConstants.CONTROLLER_PORT);

  public RobotContainer() {
    drivebase.setDefaultCommand(new EastDrive(
      drivebase,
      () -> applyDeadband(-controller.getLeftY()),
      () -> applyDeadband(-controller.getRightX()),
      () -> applyDeadband(controller.getRightTriggerAxis())
    ));
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }

  private double applyDeadband(double value) {
    double deadband = ControllerConstants.JOYSTICK_DEADBAND;

    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
