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
        new BallManager(superstructure, false),
        new TrajectorySequence(drivebase, "paths/Five_1.wpilib.json", "paths/Five_2.wpilib.json", "paths/Five_3.wpilib.json")
      ),
      new Shoot(drivebase, superstructure, vision, false).withTimeout(3),
      new InstantCommand(superstructure.intake::deploy),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new SequentialCommandGroup(
          new TrajectorySequence(drivebase, "paths/Five_4.wpilib.json"),
          new WaitCommand(1),
          new TrajectorySequence(drivebase, "paths/Five_5.wpilib.json")
        )
      ),
      new Shoot(drivebase, superstructure, vision).withTimeout(2.5)
    ));

    chooser.addOption("4 Ball", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::deploy),
      new ParallelCommandGroup(
        new BallManager(superstructure).withInterrupt(() -> {
          return superstructure.indexer.hasLowerBall() && superstructure.indexer.hasUpperBall();
        }),
        new TrajectorySequence(drivebase, "paths/Terminal_1.wpilib.json")
      ).withTimeout(6),
      new Shoot(drivebase, superstructure, vision, false).withTimeout(2.5),
      new InstantCommand(superstructure.intake::deploy),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new SequentialCommandGroup(
          new TrajectorySequence(drivebase, "paths/Terminal_2.wpilib.json"),
          new WaitCommand(1),
          new TrajectorySequence(drivebase, "paths/Terminal_3.wpilib.json")
        )
      ),
      new Shoot(drivebase, superstructure, vision).withTimeout(2.5)
    ));

    chooser.addOption("2 Ball", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::deploy),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new TrajectorySequence(drivebase, "paths/Two_1.wpilib.json")
      ),
      new Shoot(drivebase, superstructure, vision, false).withTimeout(2.5),
      new InstantCommand(superstructure.intake::deploy),
      new ParallelCommandGroup(
        new BallManager(superstructure),
        new TrajectorySequence(drivebase, "paths/Two_2.wpilib.json", "paths/Two_3.wpilib.json")
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