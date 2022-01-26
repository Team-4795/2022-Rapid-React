// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.DrivebaseTeleop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Solenoid;

public class RobotContainer {
  private final Drivebase drivebase = new Drivebase();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Vision vision = new Vision();
  private final Solenoid solenoid = new Solenoid();
  
  private final XboxController controller = new XboxController(ControllerConstants.CONTROLLER_PORT);

  public RobotContainer() {
    drivebase.setDefaultCommand(new DrivebaseTeleop(
      drivebase,
      () -> applyDeadband(-controller.getLeftY()),
      () -> applyDeadband(-controller.getRightX()),
      () -> applyDeadband(controller.getRightTriggerAxis())
    ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
  
    final JoystickButton buttonA = new JoystickButton(controller,0); //button A
    final JoystickButton buttonB = new JoystickButton(controller,1); //button B


      buttonA.whenPressed(new ParallelCommandGroup(
        new InstantCommand(() -> solenoid.extend()),
        new WaitCommand(1.0),
        new InstantCommand(() -> solenoid.off())
    ));

    buttonB.whenPressed(new ParallelCommandGroup(
      new InstantCommand(() -> solenoid.retract()),
      new WaitCommand(1.0),
      new InstantCommand(() -> solenoid.off())
  ));
  }

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
