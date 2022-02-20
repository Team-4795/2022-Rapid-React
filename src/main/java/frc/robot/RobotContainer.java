// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.TrajectorySequence;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.Shoot;
import frc.robot.commands.BallManager;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final Drivebase drivebase;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Climber climber;
  private final Vision vision;

  private final Controller controller = new Controller(ControllerConstants.CONTROLLER_PORT);

  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  public RobotContainer() {
    drivebase = new Drivebase();
    shooter = new Shooter();
    intake = new Intake();
    indexer = new Indexer();
    climber = new Climber();
    vision = new Vision();

    drivebase.setDefaultCommand(new CurvatureDrive(
      drivebase,
      () -> -controller.getLeftY(),
      () -> -controller.getRightX(),
      () -> controller.getRightTriggerAxis()
    ));
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setShooterSpeed(0, 0), shooter));

    CommandScheduler.getInstance().schedule(new BallManager(intake, indexer, shooter));

    autoSelector.setDefaultOption("Test 1", new TrajectorySequence(drivebase, "paths/Forward.wpilib.json", "paths/Reverse.wpilib.json"));
    autoSelector.addOption("Test 2", new TrajectorySequence(drivebase, "paths/OneBallPath.wpilib.json"));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final JoystickButton buttonA = new JoystickButton(controller, Controller.Button.kA.value);
    final JoystickButton buttonB = new JoystickButton(controller, Controller.Button.kB.value);
    final JoystickButton buttonY = new JoystickButton(controller, Controller.Button.kY.value);
    final JoystickButton buttonX = new JoystickButton(controller, Controller.Button.kX.value);
    final JoystickButton rightBumper = new JoystickButton(controller, Controller.Button.kRightBumper.value);

    //Run all motors (Precent Out)
    buttonA.whenHeld(new ParallelCommandGroup(
      new RunCommand(() -> intake.setSpeed(.5)),
      new RunCommand(() -> indexer.setIndexerSpeed(.5, .5)),
      new RunCommand(() -> shooter.setShooterSpeed(.1, .1)),
      new RunCommand(() -> drivebase.tankDriveVolts(0, 0))
    ));
  
    //Set Speed of Intake
    buttonB.whileHeld(new Shoot(drivebase, shooter, vision));

    //Indexer and Shooter to Shoot Ball
    buttonY.whileHeld(new ParallelCommandGroup(
      // new RunCommand(() -> shooter.setShooterRPM(5000, 6000))
      new RunCommand(() -> shooter.setShooterSpeed(.5, .5)),

      new SequentialCommandGroup(
        // new RunCommand(() -> shooter.setShooterRPM(5000, 6000))
        new WaitCommand(3),
        new RunCommand(() -> indexer.setIndexerSpeed(.5, .5)) // CHANGE VALUES LATER
      )
    ));

    //Intake and Index Ball
    buttonX.whenHeld(new ParallelCommandGroup(
      new InstantCommand(() -> intake.toggle()),
      new RunCommand(() -> indexer.setIndexerSpeed(.5, .5))
    ));

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