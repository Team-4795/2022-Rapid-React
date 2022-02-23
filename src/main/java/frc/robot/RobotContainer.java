// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.TrajectorySequence;
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
  private final Intake intake;

  private final Controller controller = new Controller(ControllerConstants.CONTROLLER_PORT);

  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  public RobotContainer() {
    drivebase = new Drivebase();
    superstructure = new Superstructure();
    shooter = new Shooter();
    climber = new Climber();
    vision = new Vision();
    intake = new Intake();

    drivebase.setDefaultCommand(new CurvatureDrive(
      drivebase,
      () -> -controller.getLeftY(),
      () -> -controller.getRightX(),
      () -> controller.getRightTriggerAxis()
    ));
    superstructure.setDefaultCommand(new BallManager(superstructure));
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setShooterSpeed(0, 0), shooter));
    vision.setDefaultCommand(new RunCommand(vision::disableLED, vision));

    autoSelector.setDefaultOption("Bue Hanger 2", new SequentialCommandGroup(new TrajectorySequence(drivebase, "paths/BlueHanger2.wpilib.json"), new InstantCommand(intake::toggle), new Shoot(drivebase, superstructure, shooter, vision)));
    autoSelector.addOption("Bue Terminal 2", new SequentialCommandGroup(new TrajectorySequence(drivebase, "paths/BlueTerminal2.wpilib.json"), new InstantCommand(intake::toggle), new Shoot(drivebase, superstructure, shooter, vision)));
    autoSelector.setDefaultOption("Red Hanger 2", new SequentialCommandGroup(new TrajectorySequence(drivebase, "paths/RedHanger2.wpilib.json"), new InstantCommand(intake::toggle), new Shoot(drivebase, superstructure, shooter, vision)));
    autoSelector.addOption("Red Terminal 2", new SequentialCommandGroup(new TrajectorySequence(drivebase, "paths/RedTerminal2.wpilib.json"), new InstantCommand(intake::toggle), new Shoot(drivebase, superstructure, shooter, vision)));

    autoSelector.addOption("Bue Terminal 3", new SequentialCommandGroup(new Shoot(drivebase, superstructure, shooter, vision), new TrajectorySequence(drivebase, "paths/BlueTerminal3.wpilib.json"), new InstantCommand(intake::toggle), new Shoot(drivebase, superstructure, shooter, vision)));
    autoSelector.addOption("Red Terminal 3", new SequentialCommandGroup(new Shoot(drivebase, superstructure, shooter, vision), new TrajectorySequence(drivebase, "paths/RedTerminal3.wpilib.json"), new InstantCommand(intake::toggle), new Shoot(drivebase, superstructure, shooter, vision)));
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final JoystickButton buttonA = new JoystickButton(controller, Controller.Button.kA.value);
    final JoystickButton buttonB = new JoystickButton(controller, Controller.Button.kB.value);
    final JoystickButton buttonX = new JoystickButton(controller, Controller.Button.kX.value);
    final JoystickButton rightBumper = new JoystickButton(controller, Controller.Button.kRightBumper.value);

    buttonA.whileHeld(new Shoot(drivebase, superstructure, shooter, vision));
    buttonB.whenPressed(superstructure.intake::toggle);
    buttonX.whenPressed(new RunCommand(climber::toggle, climber));
    rightBumper.whenPressed(drivebase::reverse);
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }
}