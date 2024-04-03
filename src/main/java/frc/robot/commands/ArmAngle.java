package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ArmAngle extends Command {

    private final double angle;

    public ArmAngle( double angle ){
        this.angle = angle;
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void initialize(){
        RobotContainer.shooter.setAngle(angle);
    }

    @Override
    public void execute() {
       // m_arm.getController().setGoal(m_angle);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
