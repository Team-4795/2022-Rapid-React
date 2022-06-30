// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.Shoot.ShooterPreset;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.GoalTracker;
import frc.robot.commands.IndependentDrive;
import frc.robot.commands.Shoot;
import frc.robot.commands.BallManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  public final Drivebase drivebase;
  public final Superstructure superstructure;
  public final Climber climber;
  public final Vision vision;

  private final AutoSelector autoSelector;

  private final BallManager ballManager;

  private final Controller driverController = new Controller(ControllerConstants.DRIVER);

  public RobotContainer() {
    drivebase = new Drivebase();
    superstructure = new Superstructure();
    climber = new Climber();
    vision = new Vision();
    autoSelector = new AutoSelector(drivebase, superstructure, vision);

    drivebase.setDefaultCommand(new IndependentDrive(
      drivebase,
      () -> driverController.getLeftY(),
      () -> driverController.getRightX()
    ));

    ballManager = new BallManager(superstructure, drivebase);
    superstructure.setDefaultCommand(ballManager);

    climber.setDefaultCommand(new RunCommand(() -> climber.setPower(climber.getPosition() > 8 && climber.getPosition() < 15 ? -1 : 0), climber));
    vision.setDefaultCommand(new GoalTracker(drivebase, vision));

    SmartDashboard.putData(drivebase);
    SmartDashboard.putData(superstructure.indexer);
    SmartDashboard.putData(superstructure.shooter);
    SmartDashboard.putData(climber);
    SmartDashboard.putData(vision);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final JoystickButton reverseButton = new JoystickButton(driverController, Controller.Button.kRightBumper.value);
    final JoystickButton lowShot = new JoystickButton(driverController, Controller.Button.kA.value);
    final JoystickButton intakeButton = new JoystickButton(driverController, Controller.Button.kB.value);

    final JoystickButton highShot = new JoystickButton(driverController, Controller.Button.kY.value);

    reverseButton.whenPressed(drivebase::reverse);
    lowShot.whileHeld(new Shoot(drivebase, superstructure, vision, new ShooterPreset(1500, 700, 0)));
    highShot.whileHeld(new Shoot(drivebase, superstructure, vision, new ShooterPreset(1800, 1200, 0)));
    intakeButton.whenPressed(superstructure.intake::toggle);
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setDriverRumble(double rumble) {
    driverController.setRumble(RumbleType.kLeftRumble, rumble);
    driverController.setRumble(RumbleType.kRightRumble, rumble);
  }
}