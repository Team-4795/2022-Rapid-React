// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.curveDrive;
import frc.robot.subsystems.Drivebase;


public class RobotContainer {

  private final Drivebase drivebase = new Drivebase();
  
  private final XboxController controller = new XboxController(ControllerConstants.CONTROLLER_PORT);

  //private final PowerDistribution PDP = new PowerDistribution();

  public RobotContainer() {
    drivebase.setDefaultCommand(new curveDrive(drivebase, () -> controller.getRawAxis(ControllerConstants.SPEED_JOYSTICK), () -> controller.getRawAxis(ControllerConstants.ROTATION_JOYSTICK), () -> controller.getRawAxis(ControllerConstants.THROTTLE_TRIGGER)));
    //PDP.clearStickyFaults();
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }

 
}
