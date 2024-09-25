package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeNote extends Command {
    public IntakeNote(){
        
        RobotContainer.shooter.aimIntakeCommand()
            .andThen( RobotContainer.intake.runCommand().alongWith(RobotContainer.shooter.intakeNoteCommand()))
            .until( RobotContainer.intake::isLoaded );
            
        addRequirements(RobotContainer.intake, RobotContainer.shooter);
    }
}
