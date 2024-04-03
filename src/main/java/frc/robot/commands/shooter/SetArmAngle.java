package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetArmAngle extends Command {

    private final double angle;

    public SetArmAngle( double angle ){
        this.angle = angle;
        addRequirements(RobotContainer.shooter.arm);
    }

    @Override
    public void initialize(){
        RobotContainer.shooter.arm.setAngle(angle);
    }

    @Override
    public boolean isFinished(){
        return RobotContainer.shooter.arm.getController().atGoal();
    }
}
