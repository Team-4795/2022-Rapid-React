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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.GoalTracker;
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
  private final Controller operatorController = new Controller(ControllerConstants.OPERATOR);

  public RobotContainer() {
    drivebase = new Drivebase();
    superstructure = new Superstructure();
    climber = new Climber();
    vision = new Vision();
    autoSelector = new AutoSelector(drivebase, superstructure, vision);

    drivebase.setDefaultCommand(new CurvatureDrive(
      drivebase,
      () -> -driverController.getLeftY(),
      () -> driverController.getRightX(),
      () -> driverController.getRightTriggerAxis()
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
    final JoystickButton reverseButton = new JoystickButton(driverController, 6);
    final JoystickButton shootButton = new JoystickButton(driverController, 1);
    final JoystickButton intakeButton = new JoystickButton(driverController, 2);
    // final JoystickButton driveToGoal = new JoystickButton(driverController, 3);
    final JoystickButton lowGoalButton = new JoystickButton(driverController, 4);

    final JoystickButton unjamButton = new JoystickButton(operatorController, 2);
    final JoystickButton intakeOverride = new JoystickButton(operatorController, 3);
    final JoystickButton resetClimber = new JoystickButton(operatorController, 1);
    final JoystickButton manualRetract = new JoystickButton(operatorController, 4);
    final Trigger reverseIntake = new Trigger(() -> operatorController.getRightTriggerAxis() > 0);
    final Trigger tiltClimber = new Trigger(() -> operatorController.getPOV() == 0);
    final Trigger untiltClimber = new Trigger(() -> operatorController.getPOV() == 180);
    final JoystickButton extendClimber = new JoystickButton(operatorController, 6);
    final JoystickButton retractClimber = new JoystickButton(operatorController, 5);

    reverseButton.whenPressed(drivebase::reverse);
    shootButton.whileHeld(new Shoot(drivebase, superstructure, vision));
    // driveToGoal.whileHeld(new SequentialCommandGroup(new PrepareShot(drivebase, superstructure, vision), new Shoot(drivebase, superstructure, vision)));
    lowGoalButton.whileHeld(new Shoot(drivebase, superstructure, vision, new ShooterPreset(1500, 750, 0)));
    intakeButton.whenPressed(superstructure.intake::toggle);

    unjamButton.whileHeld(new RunCommand(() -> {
      superstructure.indexer.setIndexerSpeed(-0.3, -1);
      superstructure.shooter.setShooterPower(-1, -0.3);
    }, superstructure));
    intakeOverride.whenPressed(superstructure.intake::toggle);
    reverseIntake.whenActive(() -> ballManager.setIntakeReversed(true)).whenInactive(() -> ballManager.setIntakeReversed(false));
    resetClimber.whenPressed(climber::resetEncoder);
    manualRetract.whileHeld(new RunCommand(() -> climber.setPower(-0.2), climber));
    tiltClimber.whenActive(climber::tilt);
    untiltClimber.whenActive(climber::untilt);
    extendClimber.whileHeld(new RunCommand(climber::extend, climber));
    retractClimber.whileHeld(new RunCommand(climber::retract, climber));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setDriverRumble(double rumble) {
    driverController.setRumble(RumbleType.kLeftRumble, rumble);
    driverController.setRumble(RumbleType.kRightRumble, rumble);
  }

  public void setOperatorRumble(double rumble) {
    operatorController.setRumble(RumbleType.kLeftRumble, rumble);
    operatorController.setRumble(RumbleType.kRightRumble, rumble);
  }
}