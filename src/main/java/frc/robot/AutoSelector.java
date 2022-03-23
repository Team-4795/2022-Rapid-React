package frc.robot;

import javax.swing.text.html.ParagraphView;

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
    chooser.setDefaultOption("3 Ball + Human Ball", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::toggle),
      new ParallelRaceGroup(
        new ParallelCommandGroup(
          new BallManager(superstructure),
          new TrajectorySequence(drivebase, "paths/3 Ball Human.wpilib.json")),
        new WaitCommand(2.5)),
      new ParallelRaceGroup(
        new Shoot(drivebase, superstructure, vision),
        new WaitCommand(3)),
      new InstantCommand(superstructure.intake::toggle),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new TrajectorySequence(drivebase, "paths/3 Ball Human_0.wpilib.json")),
      new SequentialCommandGroup(
        new BallManager(superstructure),
        new WaitCommand(2)
      ),
      new TrajectorySequence(drivebase, "paths/3 Ball Human_1.wpilib.json"),
      new ParallelRaceGroup(
        new Shoot(drivebase, superstructure, vision),
        new WaitCommand(3)
      )
    ));

    chooser.addOption("Backup", new SequentialCommandGroup(
      new InstantCommand(superstructure.intake::toggle),
      new ParallelRaceGroup(
        new BallManager(superstructure),
        new RunCommand(() -> drivebase.curvatureDrive(0.35, 0, false), drivebase),
        new WaitCommand(3)
      ),
      new Shoot(drivebase, superstructure, vision)
      )
    );
    
    SmartDashboard.putData(chooser);
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}