package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ScoreSpeaker extends Command {
    
    public ScoreSpeaker(){

        RobotContainer.shooter.arm.setAngleCommand(Constants.Shooter.Arm.SPEAKER_ANGLE_SHORT);
        addRequirements(RobotContainer.shooter);
    }

}
