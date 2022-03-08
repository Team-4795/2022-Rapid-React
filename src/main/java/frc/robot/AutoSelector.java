package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallManager;
import frc.robot.commands.Shoot;
import frc.robot.commands.TrajectorySequence;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AutoSelector {
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public AutoSelector(Drivebase drivebase, Superstructure superstructure, Shooter shooter, Vision vision) {
    // chooser.setDefaultOption("Blue Hanger 2", new SequentialCommandGroup(
    //   new InstantCommand(superstructure.intake::toggle),
    //   new ParallelRaceGroup(
    //     new TrajectorySequence(drivebase, "paths/BlueHanger2.wpilib.json"),
    //     new BallManager(superstructure)
    //   ),
    //   new Shoot(drivebase, superstructure, shooter, vision)
    //   ));
    // chooser.addOption("Blue Terminal 2", new SequentialCommandGroup(
    //   new InstantCommand(superstructure.intake::toggle),
    //   new ParallelRaceGroup(
    //     new TrajectorySequence(drivebase, "paths/BlueTerminal2.wpilib.json"),
    //     new BallManager(superstructure)
    //   ),
    //   new Shoot(drivebase, superstructure, shooter, vision)
    //   ));
    chooser.addOption("3 Ball", new SequentialCommandGroup(
      new ParallelRaceGroup(new Shoot(drivebase, superstructure, shooter, vision), new WaitCommand(2)),
      new InstantCommand(superstructure.intake::toggle),
      new ParallelRaceGroup(
        new TrajectorySequence(drivebase, "paths/3BallAuto_V2.wpilib.json"),
        new BallManager(superstructure)
      ),
      new Shoot(drivebase, superstructure, shooter, vision)));

    // chooser.addOption("Red Hanger 2", new SequentialCommandGroup(
    //   new InstantCommand(superstructure.intake::toggle),
    //   new ParallelRaceGroup(
    //     new TrajectorySequence(drivebase, "paths/RedHanger2.wpilib.json"),
    //     new BallManager(superstructure)
    //     ),
    //   new Shoot(drivebase, superstructure, shooter, vision)));
    // chooser.addOption("Red Terminal 2", new SequentialCommandGroup(
    //   new InstantCommand(superstructure.intake::toggle),
    //   new ParallelRaceGroup(
    //     new TrajectorySequence(drivebase, "paths/RedTerminal2.wpilib.json"),
    //     new BallManager(superstructure)
    //   ),
    //   new Shoot(drivebase, superstructure, shooter, vision)));
    // chooser.addOption("Red Terminal 3", new SequentialCommandGroup(
    //   new ParallelRaceGroup(new Shoot(drivebase, superstructure, shooter, vision), new WaitCommand(3)),
    //   new InstantCommand(superstructure.intake::toggle),
    //   new ParallelRaceGroup(
      //   new TrajectorySequence(drivebase, "paths/RedTerminal3.wpilib.json"),
      //   new BallManager(superstructure)
      // ),
      // new Shoot(drivebase, superstructure, shooter, vision)));

    chooser.addOption("Forwards Backwards", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::toggle),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new TrajectorySequence(drivebase, "paths/Forward.wpilib.json", "paths/Reverse.wpilib.json")),
      new Shoot(drivebase, superstructure, shooter, vision))
    );

    chooser.addOption("Backup", new ParallelRaceGroup(
      new RunCommand(() -> drivebase.curvatureDrive(-0.35, 0, false), drivebase),
      new WaitCommand(3)));
    
    SmartDashboard.putData(chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}