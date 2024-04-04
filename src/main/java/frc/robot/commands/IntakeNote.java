package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ArmAngle;

public class IntakeNote extends SequentialCommandGroup {

    public IntakeNote(){
        
        addCommands(

        );

        addRequirements(RobotContainer.intake, RobotContainer.shooter);
    }

}
