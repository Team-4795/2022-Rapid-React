package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;

public class TrajectorySequence extends SequentialCommandGroup {
  public TrajectorySequence(Drivebase drivetrain, String ... paths) {

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
    return null;
  }
}