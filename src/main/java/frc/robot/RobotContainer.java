// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final Controller driverController = new Controller(ControllerConstants.DRIVER);
  private final Controller operatorController = new Controller(ControllerConstants.OPERATOR);

  private final SendableChooser<Command> autoSelector = new SendableChooser<>();

  public RobotContainer() {
    drivebase = new Drivebase();
    superstructure = new Superstructure();
    shooter = new Shooter();
    climber = new Climber();
    vision = new Vision();

    drivebase.setDefaultCommand(new CurvatureDrive(
      drivebase,
      () -> -driverController.getLeftY(),
      () -> driverController.getRightX(),
      () -> driverController.getRightTriggerAxis()
    ));
    superstructure.setDefaultCommand(new BallManager(superstructure));
    shooter.setDefaultCommand(new RunCommand(() -> shooter.setShooterPower(0, 0), shooter));
    vision.setDefaultCommand(new RunCommand(vision::disableLED, vision));

    autoSelector.setDefaultOption("Blue Hanger 2", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::toggle),
      new TrajectorySequence(drivebase, "paths/BlueHanger2.wpilib.json"),
      new Shoot(drivebase, superstructure, shooter, vision)
      ));
    autoSelector.addOption("Blue Terminal 2", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::toggle),
      new TrajectorySequence(drivebase, "paths/BlueTerminal2.wpilib.json"),
      new Shoot(drivebase, superstructure, shooter, vision)
      ));
    autoSelector.addOption("Blue Terminal 3", new SequentialCommandGroup(
      new ParallelRaceGroup(new Shoot(drivebase, superstructure, shooter, vision), new WaitCommand(3)),
      new InstantCommand(superstructure.intake::toggle),
      new TrajectorySequence(drivebase, "paths/BlueTerminal3.wpilib.json"),
      new Shoot(drivebase, superstructure, shooter, vision)));

    autoSelector.addOption("Red Hanger 2", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::toggle),
      new TrajectorySequence(drivebase, "paths/RedHanger2.wpilib.json"),
      new Shoot(drivebase, superstructure, shooter, vision)));
    autoSelector.addOption("Red Terminal 2", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::toggle),
      new TrajectorySequence(drivebase, "paths/RedTerminal2.wpilib.json"),
      new Shoot(drivebase, superstructure, shooter, vision)));
    autoSelector.addOption("Red Terminal 3", new SequentialCommandGroup(
      new ParallelRaceGroup(new Shoot(drivebase, superstructure, shooter, vision), new WaitCommand(3)),
      new InstantCommand(superstructure.intake::toggle),
      new TrajectorySequence(drivebase, "paths/RedTerminal3.wpilib.json"),
      new Shoot(drivebase, superstructure, shooter, vision)));

    autoSelector.addOption("Backup", new ParallelRaceGroup(
      new RunCommand(() -> drivebase.curvatureDrive(-0.35, 0, false), drivebase),
      new WaitCommand(3)));
    
    SmartDashboard.putData(autoSelector);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final JoystickButton reverseButton = new JoystickButton(driverController, Controller.Button.kRightBumper.value);
    final JoystickButton shootButton = new JoystickButton(driverController, Controller.Button.kA.value);
    final JoystickButton intakeButton = new JoystickButton(driverController, Controller.Button.kB.value);
    final JoystickButton climbButton = new JoystickButton(driverController, Controller.Button.kX.value);
    final JoystickButton lowGoalButton = new JoystickButton(driverController, Controller.Button.kY.value);
    final JoystickButton unjamButton = new JoystickButton(operatorController, Controller.Button.kA.value);

    reverseButton.whenPressed(drivebase::reverse);
    shootButton.whileHeld(new Shoot(drivebase, superstructure, shooter, vision));
    intakeButton.whenPressed(superstructure.intake::toggle);
    climbButton.whenPressed(climber::toggle);
    lowGoalButton.whileHeld(new ParallelCommandGroup(
      new RunCommand(() -> shooter.setShooterPower(0.3, 0.3), shooter),
      new SequentialCommandGroup(
        new WaitCommand(2),
        new RunCommand(() -> superstructure.indexer.setIndexerSpeed(0.5, 1), superstructure)
      )
    ));
    unjamButton.whileHeld(new RunCommand(() -> {
      superstructure.indexer.setIndexerSpeed(-0.3, -0.3);
      shooter.setShooterPower(-0.3, 0);
    }, superstructure, shooter));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setRumble(double rumble) {
    driverController.setRumble(RumbleType.kLeftRumble, rumble);
    driverController.setRumble(RumbleType.kRightRumble, rumble);
  }
}