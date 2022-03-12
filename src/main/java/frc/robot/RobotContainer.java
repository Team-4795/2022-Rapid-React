// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.Shoot;
import frc.robot.commands.BallManager;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final Drivebase drivebase;
  private final Superstructure superstructure;
  private final Shooter shooter;
  private final Climber climber;
  private final Vision vision;
  private final AutoSelector autoSelector;

  private final Controller driverController = new Controller(ControllerConstants.DRIVER);
  private final Controller operatorController = new Controller(ControllerConstants.OPERATOR);

  public RobotContainer() {
    drivebase = new Drivebase();
    superstructure = new Superstructure();
    shooter = new Shooter();
    climber = new Climber();
    vision = new Vision();
    autoSelector = new AutoSelector(drivebase, superstructure, shooter, vision);

    drivebase.setDefaultCommand(new CurvatureDrive(
      drivebase,
      () -> -driverController.getLeftY(),
      () -> driverController.getRightX(),
      () -> driverController.getRightTriggerAxis()
    ));
    superstructure.setDefaultCommand(new BallManager(superstructure));
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setShooterPower(0, 0), shooter));
    climber.setDefaultCommand(new RunCommand(() -> climber.setPower(0), climber));
    vision.setDefaultCommand(new RunCommand(vision::disableLED, vision));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final JoystickButton reverseButton = new JoystickButton(driverController, Controller.Button.kRightBumper.value);
    final JoystickButton shootButton = new JoystickButton(driverController, Controller.Button.kA.value);
    final JoystickButton intakeButton = new JoystickButton(driverController, Controller.Button.kB.value);

    final JoystickButton unjamButton = new JoystickButton(operatorController, Controller.Button.kA.value);
    final JoystickButton intakeOverride = new JoystickButton(operatorController, Controller.Button.kB.value);
    final JoystickButton resetClimber = new JoystickButton(operatorController, Controller.Button.kX.value);
    final JoystickButton manualRetract = new JoystickButton(operatorController, Controller.Button.kY.value);
    final JoystickButton retractClimber = new JoystickButton(operatorController, Controller.Button.kLeftBumper.value);
    final JoystickButton extendClimber = new JoystickButton(operatorController, Controller.Button.kRightBumper.value);

    reverseButton.whenPressed(drivebase::reverse);
    shootButton.whileHeld(new Shoot(drivebase, superstructure, shooter, vision));
    intakeButton.whenPressed(superstructure.intake::toggle);
    retractClimber.whileHeld(new RunCommand(climber::retract, climber));
    extendClimber.whileHeld(new RunCommand(climber::extend, climber));

    unjamButton.whileHeld(new RunCommand(() -> {
      superstructure.indexer.setIndexerSpeed(-0.3, -0.3);
      shooter.setShooterPower(-0.3, 0);
    }, superstructure, shooter));
    intakeOverride.whenPressed(superstructure.intake::toggle);
    resetClimber.whenPressed(climber::resetEncoder);
    manualRetract.whileHeld(new RunCommand(() -> climber.setPower(-0.2), climber));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setRumble(double rumble) {
    driverController.setRumble(RumbleType.kLeftRumble, rumble);
    driverController.setRumble(RumbleType.kRightRumble, rumble);
  }
}