package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.ControllerConstants;

public class Controller extends XboxController {
  public Controller(int port) {
    super(port);
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

  @Override
  public double getLeftX() {
    return applyDeadband(super.getLeftX());
  }

  @Override
  public double getLeftY() {
    return applyDeadband(super.getLeftY());
  }

  @Override
  public double getRightX() {
    return applyDeadband(super.getRightX());
  }

  @Override
  public double getRightY() {
    return applyDeadband(super.getRightY());
  }

  @Override
  public double getLeftTriggerAxis() {
    return applyDeadband(super.getLeftTriggerAxis());
  }

  @Override
  public double getRightTriggerAxis() {
    return applyDeadband(super.getRightTriggerAxis());
  }
}