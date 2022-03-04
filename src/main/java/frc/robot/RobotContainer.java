// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.curveDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Climber;


public class RobotContainer {

  private final Drivebase drivebase = new Drivebase();
  private final Climber climber = new Climber();
  
  private final XboxController controller = new XboxController(ControllerConstants.CONTROLLER_PORT);

  //private final PowerDistribution PDP = new PowerDistribution();

  public RobotContainer() {
    
    drivebase.setDefaultCommand(new curveDrive(drivebase, () -> -controller.getRawAxis(ControllerConstants.SPEED_JOYSTICK), () -> controller.getRawAxis(ControllerConstants.ROTATION_JOYSTICK), () -> controller.getRawButton(ControllerConstants.ROTATE_IN_PLACE_BUTTON), () -> controller.getRawAxis(ControllerConstants.THROTTLE_TRIGGER)));


    drivebase.setDefaultCommand(
      new curveDrive(drivebase,
      () -> applyDeadband(-controller.getLeftY()),
      () -> applyDeadband(controller.getRightX()),
      () -> controller.getRightBumper(),
      () -> applyDeadband(controller.getRightTriggerAxis()))
    );
    //PDP.clearStickyFaults();

    climber.setDefaultCommand(new InstantCommand(() -> climber.set(controller.getRawAxis(ControllerConstants.CLIMBER_JOYSTICK))));

    configureButtonBindings();

  }




  private void configureButtonBindings() {
  
    final JoystickButton buttonX = new JoystickButton(controller,3); //button X
    final JoystickButton buttonY = new JoystickButton(controller,4); //button Y

    
    buttonX.whenPressed(
      new InstantCommand(() -> climber.extend())
    );

    buttonY.whenPressed(
      new InstantCommand(() -> climber.retract())
    );

  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }

  public double applyDeadband(double value) {
    double deadband = ControllerConstants.JOYSTICK_DEADBAND;

    if (Math.abs(value) < deadband) {
      return 0;
    } else {
      return value;
    }
  }
 
}
