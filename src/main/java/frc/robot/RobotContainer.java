// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.TrajectorySequence;
import frc.robot.commands.curveDrive;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private Drivebase drivebase;
  private Shooter shooter;
  private Intake intake;
  private Indexer indexer;

  private final XboxController controller = new XboxController(ControllerConstants.CONTROLLER_PORT);

  SendableChooser<Command> autoSelector = new SendableChooser<>();

  public RobotContainer() {
    drivebase = new Drivebase();
    shooter = new Shooter();
    intake = new Intake();
    indexer = new Indexer();

    autoSelector.setDefaultOption("Test 1", new TrajectorySequence(drivebase, "paths/Forward.wpilib.json", "paths/Reverse.wpilib.json"));
    autoSelector.addOption("Test 2", new TrajectorySequence(drivebase, "paths/OneBallPath.wpilib.json"));

    drivebase.setDefaultCommand(new curveDrive(drivebase,
    () -> -controller.getRawAxis(ControllerConstants.SPEED_JOYSTICK),
    () -> controller.getRawAxis(ControllerConstants.ROTATION_JOYSTICK),
    () -> controller.getRawButton(ControllerConstants.ROTATE_IN_PLACE_BUTTON),
    () -> controller.getRawAxis(ControllerConstants.THROTTLE_TRIGGER)));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    final JoystickButton buttonA = new JoystickButton(controller,0); //button A
    final JoystickButton buttonB = new JoystickButton(controller,1); //button B
    final JoystickButton buttonY = new JoystickButton(controller,3); //button Y CHECK BINDING FOR THIS, PROB NOT THREE
    final JoystickButton buttonX = new JoystickButton(controller,4); //button X CHECK BINDING FOR THIS, PROB NOT THREE

    //Extend and Spin Spinner of Intake
    buttonA.whenPressed(new ParallelCommandGroup(
      new InstantCommand(() -> intake.toggleIntake()),
      new InstantCommand(() -> intake.setSpeed(.5))
    ));
  
    //Set Speed of Intake
    buttonB.whenPressed(new ParallelCommandGroup(
      new InstantCommand(() -> intake.setSpeed(0))
    ));

    //Indexer and Shooter to Shoot Ball
    buttonY.whileHeld(new SequentialCommandGroup(
      new InstantCommand(() -> indexer.setIndexerSpeed(.5, .5)), // CHANGE VALUES LATER
      new WaitCommand(3),
      new InstantCommand(() -> shooter.setShooterRPM(5000, 6000)) // CHANGE VALUES LATER
    ));

    //Intake and Index Ball
    buttonX.whileHeld(new ParallelCommandGroup(
      new InstantCommand(() -> intake.toggleIntake()),
      new InstantCommand(() -> indexer.setIndexerSpeed(.5, .5))
    ));

    //THESE ARE NO LONGER NEEDED
    Joystick exampleStick = new Joystick(1); // Creates a joystick on port 1
    JoystickButton exampleButton = new JoystickButton(exampleStick, 1); // Creates a new JoystickButton object for button 1 on exampleStick
    exampleButton.whenPressed(new InstantCommand(() -> shooter.setShooterRPM(0, 0)));
  }

  public Command getAutonomousCommand() {
    return autoSelector.getSelected();
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }
}