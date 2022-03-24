// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbModular extends SequentialCommandGroup {
	
	// @param Climber

  //BEGIN LINED UP WITH MID RUNG BEFORE ANY CLIMB

  private double timeToRetract = 1.25;
  private double timeToExtend = 1;
  private double swingDelay = 1.25;
  //Delay between fully retracting onto high rung and starting extending towards traverse rung

	public ClimbModular(Climber climber) {
		switch (climber.level) {
      case START:
        addCommands(
          new RunCommand(climber::advanceStage, climber)
        );
      case MIDRETRACTING:
        addCommands(
			    new ParallelRaceGroup(
			    	new RunCommand(climber::retract, climber),
			    	new WaitCommand(timeToRetract),
            new RunCommand(climber::advanceStage, climber)
	    		));
      case HIGHEXTENDING:
		  	addCommands(
          new ParallelRaceGroup(
	  		  	new RunCommand(climber::extend, climber),
	  			  new WaitCommand(timeToExtend),
            new RunCommand(climber::advanceStage, climber)
	  		));
      case HIGHRETRACTING:
        addCommands(
    			new ParallelRaceGroup(
		    		new RunCommand(climber::retract, climber),
			    	new WaitCommand(timeToRetract),
            new RunCommand(climber::advanceStage, climber)
			  ));
			case HIGHWAITING:
        addCommands(new WaitCommand(swingDelay),
        new RunCommand(climber::advanceStage, climber)
      );  
      case TRAVERSEEXTENDING:
        addCommands(
          new ParallelRaceGroup(
				    new RunCommand(climber::extend, climber),
				    new WaitCommand(timeToRetract),
            new RunCommand(climber::advanceStage, climber)
			  ));
      case TRAVERSERETRACTING:
        addCommands(
    			new ParallelRaceGroup(
		    		new RunCommand(climber::retract, climber),
			    	new WaitCommand(timeToRetract),
            new RunCommand(climber::advanceStage, climber)
			  ));
      
      case END:
        addCommands(
          //idk what to put here lol
        );
    }
	}

}

