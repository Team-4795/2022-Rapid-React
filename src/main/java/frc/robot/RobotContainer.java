// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.curveDrive;
import frc.robot.subsystems.Shooter;


public class RobotContainer {

    private Drivebase drivebase;
    private Shooter shooter;
    private Intake intake;
    private Indexer indexer;

    private final XboxController controller = new XboxController(ControllerConstants.CONTROLLER_PORT);

    //private final PowerDistribution PDP = new PowerDistribution();

    public RobotContainer() {
      drivebase = new Drivebase();
      shooter = new Shooter();
      intake = new Intake();
      indexer = new Indexer();

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

    public Command generatePath(String pathName) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathName);
        Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        RamseteCommand ramseteCommand = new RamseteCommand(
          trajectory,
          drivebase::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(
            DrivebaseConstants.ksVolts,
            DrivebaseConstants.kvVoltSecondsPerMeter,
            DrivebaseConstants.kaVoltSecondsSquaredPerMeter),
          DrivebaseConstants.kDriveKinematics,
          drivebase::getWheelSpeeds,
          new PIDController(DrivebaseConstants.kPDriveVel, 0, 0),
          new PIDController(DrivebaseConstants.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          drivebase::tankDriveVolts,
          drivebase);
        // Create and push Field2d to SmartDashboard.
        Field2d m_field = new Field2d();
        SmartDashboard.putData(m_field);

        // Push the trajectory to Field2d.
        m_field.getObject("traj").setTrajectory(trajectory);

        // Reset odometry to the starting pose of the trajectory.
        drivebase.resetOdometry(trajectory.getInitialPose());
        drivebase.putTrajectory(trajectory);
        // Set up a sequence of commands
        // First, we want to reset the drivetrain odometry
        return new InstantCommand(() -> drivebase.resetOdometry(trajectory.getInitialPose()), drivebase)
            // next, we run the actual ramsete command
            .andThen(ramseteCommand)
            // Finally, we make sure that the robot stops
            .andThen(new InstantCommand(() -> drivebase.tankDriveVolts(0, 0), drivebase));
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
      }
      return null;
    }

    public Command getAutonomousCommand() {
      return new SequentialCommandGroup(
        generatePath("paths/Forward.wpilib.json"),
        new WaitCommand(3),
        generatePath("paths/Reverse.wpilib.json")
      );
    }

    public void setRumble(double rumble) {
      controller.setRumble(RumbleType.kLeftRumble, rumble);
      controller.setRumble(RumbleType.kRightRumble, rumble);
    }
}