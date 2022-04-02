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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CurvatureDrive;
import frc.robot.commands.DriveToGoal;
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

    ballManager = new BallManager(superstructure);
    superstructure.setDefaultCommand(ballManager);

    climber.setDefaultCommand(new RunCommand(() -> climber.setPower(0), climber));
    vision.setDefaultCommand(new RunCommand(vision::disableLED, vision));

    SmartDashboard.putData(drivebase);
    SmartDashboard.putData(superstructure.indexer);
    SmartDashboard.putData(superstructure.shooter);
    SmartDashboard.putData(climber);
    SmartDashboard.putData(vision);

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final JoystickButton reverseButton = new JoystickButton(driverController, Controller.Button.kRightBumper.value);
    final JoystickButton shootButton = new JoystickButton(driverController, Controller.Button.kA.value);
    final JoystickButton intakeButton = new JoystickButton(driverController, Controller.Button.kB.value);
    final JoystickButton driveToGoal = new JoystickButton(driverController, Controller.Button.kX.value);
    final JoystickButton lowGoalButton = new JoystickButton(driverController, Controller.Button.kY.value);

    final JoystickButton unjamButton = new JoystickButton(operatorController, Controller.Button.kA.value);
    final JoystickButton intakeOverride = new JoystickButton(operatorController, Controller.Button.kB.value);
    final JoystickButton resetClimber = new JoystickButton(operatorController, Controller.Button.kX.value);
    final JoystickButton manualRetract = new JoystickButton(operatorController, Controller.Button.kY.value);
    final Trigger reverseIntake = new Trigger(() -> operatorController.getRightTriggerAxis() > 0);
    final Trigger extendClimber = new Trigger(() -> operatorController.getLeftY() > 0);
    final Trigger retractClimber = new Trigger(() -> operatorController.getLeftY() < 0);
    final JoystickButton tiltClimber = new JoystickButton(operatorController, Controller.Button.kRightBumper.value);
    final JoystickButton untiltClimber = new JoystickButton(operatorController, Controller.Button.kLeftBumper.value);

    reverseButton.whenPressed(drivebase::reverse);
    shootButton.whileHeld(new Shoot(drivebase, superstructure, vision));
    driveToGoal.whileHeld(new SequentialCommandGroup(new DriveToGoal(drivebase), new Shoot(drivebase, superstructure, vision)));
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
    extendClimber.whenActive(new RunCommand(climber::extend, climber));
    retractClimber.whenActive(new RunCommand(climber::retract, climber));
    tiltClimber.whenPressed(climber::tilt);
    untiltClimber.whenPressed(climber::untilt);
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