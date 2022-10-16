package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BallManager;
import frc.robot.commands.Shoot;
import frc.robot.commands.TrajectorySequence;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

public class AutoSelector {
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public AutoSelector(Drivebase drivebase, Superstructure superstructure, Vision vision) {
    chooser.setDefaultOption("5 Ball", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::deploy),
      new ParallelRaceGroup(
        new BallManager(superstructure, drivebase),
        new TrajectorySequence(drivebase, "paths/output/Five_1.wpilib.json", "paths/output/Five_2.wpilib.json", "paths/output/Five_3.wpilib.json")
      ),
      new ParallelCommandGroup(
        new Shoot(drivebase, superstructure, vision, false).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(0.75),
          new InstantCommand(superstructure.intake::deploy)
        )
      ),
      new ParallelRaceGroup(
        new BallManager(superstructure, drivebase),
        new SequentialCommandGroup(
          new TrajectorySequence(drivebase, "paths/output/Five_4.wpilib.json"),
          new WaitCommand(0.75),
          new TrajectorySequence(drivebase, "paths/output/Five_5.wpilib.json")
        )
      ),
      new Shoot(drivebase, superstructure, vision).withTimeout(2)
    ));

    chooser.addOption("4 Ball", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::deploy),
      new ParallelCommandGroup(
        new BallManager(superstructure).withInterrupt(() -> {
          return superstructure.indexer.hasLowerBall() && superstructure.indexer.hasUpperBall();
        }).withTimeout(4),
        new TrajectorySequence(drivebase, "paths/output/Terminal_1.wpilib.json")
      ),
      new Shoot(drivebase, superstructure, vision, false).withTimeout(2.5),
      new InstantCommand(superstructure.intake::deploy),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new SequentialCommandGroup(
          new TrajectorySequence(drivebase, "paths/output/Terminal_2.wpilib.json"),
          new WaitCommand(1),
          new TrajectorySequence(drivebase, "paths/output/Terminal_3.wpilib.json")
        )
      ),
      new Shoot(drivebase, superstructure, vision).withTimeout(5)
    ));

    chooser.addOption("2 Ball", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::deploy),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new TrajectorySequence(drivebase, "paths/output/Two_1.wpilib.json")
      ),
      new Shoot(drivebase, superstructure, vision, false).withTimeout(2.5),
      new InstantCommand(superstructure.intake::deploy),
      new ParallelCommandGroup(
        new BallManager(superstructure),
        new TrajectorySequence(drivebase, "paths/output/Two_2.wpilib.json", "paths/output/Two_3.wpilib.json")
      )
    ));

    chooser.addOption("1 Ball", new SequentialCommandGroup(
      new Shoot(drivebase, superstructure, vision, false).withTimeout(4),
      new InstantCommand(superstructure.intake::deploy),
      new ParallelCommandGroup(
        new BallManager(superstructure),
        new TrajectorySequence(drivebase, "paths/output/One-1.wpilib.json", "paths/output/One-2.wpilib.json")
      )
    ));

    chooser.addOption("Drive Back", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::deploy),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new RunCommand(() -> drivebase.curvatureDrive(0.35, 0, false), drivebase),
        new WaitCommand(2)
      ),
      new Shoot(drivebase, superstructure, vision).withTimeout(4)
    ));
    
    SmartDashboard.putData(chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}