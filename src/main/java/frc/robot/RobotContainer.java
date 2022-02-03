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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.curveDrive;
import frc.robot.subsystems.Drivebase;


public class RobotContainer {

    private final Drivebase drivebase = new Drivebase();

    private final XboxController controller = new XboxController(ControllerConstants.CONTROLLER_PORT);
    private final SendableChooser < Command > m_chooser = new SendableChooser < > ();

    //private final PowerDistribution PDP = new PowerDistribution();

    public RobotContainer() {
        drivebase.setDefaultCommand(new curveDrive(drivebase, () -> -controller.getRawAxis(ControllerConstants.SPEED_JOYSTICK) * 0.3, () -> controller.getRawAxis(ControllerConstants.ROTATION_JOYSTICK) * 0.3, () -> controller.getRawButton(ControllerConstants.ROTATE_IN_PLACE_BUTTON), () -> controller.getRawAxis(ControllerConstants.THROTTLE_TRIGGER)));
        //PDP.clearStickyFaults();
        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

    public Command generatePath(String pathName) {
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



            String trajectoryJSON = pathName;

            try {
                  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
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

            }catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
             }
             return null;
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(generatePath("paths/Testing.wpilib.json"), new WaitCommand(3), generatePath("paths/Testing_0.wpilib.json"));
        //return  generateRamseteCommand();
    }

    public void setRumble(double rumble) {
        controller.setRumble(RumbleType.kLeftRumble, rumble);
        controller.setRumble(RumbleType.kRightRumble, rumble);
    }



}