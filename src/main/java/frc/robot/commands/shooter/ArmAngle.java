package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ArmAngle extends Command {

    private final double angle;

    public ArmAngle( double angle ){
        this.angle = angle;
        addRequirements(RobotContainer.shooter.arm);
    }

    @Override
    public void initialize(){
        RobotContainer.shooter.arm.setGoal(angle);
        RobotContainer.shooter.arm.enable();
    }

    @Override
    public boolean isFinished(){
        return RobotContainer.shooter.arm.getController().atGoal();
    }
}
