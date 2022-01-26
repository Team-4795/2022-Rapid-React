// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
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
    drivebase.setDefaultCommand(new curveDrive(drivebase, () -> -controller.getRawAxis(ControllerConstants.SPEED_JOYSTICK), () -> controller.getRawAxis(ControllerConstants.ROTATION_JOYSTICK), () -> controller.getRawButton(ControllerConstants.ROTATE_IN_PLACE_BUTTON), () -> controller.getRawAxis(ControllerConstants.THROTTLE_TRIGGER)));
    //PDP.clearStickyFaults();
    configureButtonBindings();
  }

  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
              DrivebaseConstants.ksVolts,
              DrivebaseConstants.kvVoltSecondsPerMeter,
              DrivebaseConstants.kaVoltSecondsSquaredPerMeter),
              DrivebaseConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DrivebaseConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
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

    // Reset odometry to the starting pose of the trajectory.
    drivebase.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drivebase.tankDriveVolts(0, 0));
  }

  public void setRumble(double rumble) {
    controller.setRumble(RumbleType.kLeftRumble, rumble);
    controller.setRumble(RumbleType.kRightRumble, rumble);
  }


 
}
