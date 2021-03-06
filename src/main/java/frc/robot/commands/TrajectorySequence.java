package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants.AutoConstants;

public class TrajectorySequence extends SequentialCommandGroup {
  private final Drivebase drivebase;

  public TrajectorySequence(Drivebase drivetrain, String ... paths) {
    drivebase = drivetrain;

    for (String pathLocation : paths) {
      Command path = generatePath(pathLocation);
      if (path == null) return;
    }

    for (String pathLocation : paths) {
      Command path = generatePath(pathLocation);
      addCommands(path);
    }
  }

  private Command generatePath(String pathName) {
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

      // Reset odometry to the starting pose of the trajectory.
      drivebase.resetOdometry(trajectory.getInitialPose());
      // Set up a sequence of commands
      // First, we want to reset the drivetrain odometry
      return new InstantCommand(() -> drivebase.resetOdometry(trajectory.getInitialPose()), drivebase)
        .andThen(new InstantCommand(() -> drivebase.enableBrakeMode(), drivebase))
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)
        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> drivebase.tankDriveVolts(0, 0), drivebase));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathName, ex.getStackTrace());
    }
    return null;
  }
}