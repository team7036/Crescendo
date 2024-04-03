package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SpinupFlywheels extends Command {

    private final double rpm;

    public SpinupFlywheels( double rpm ){
        this.rpm = rpm;
        addRequirements(RobotContainer.shooter.flyWheels);
    }

    @Override
    public void initialize(){
        RobotContainer.shooter.flyWheels.setSpeed(rpm);
    }

    @Override
    public boolean isFinished(){
        return RobotContainer.shooter.flyWheels.atSpeed(rpm);
    }
}
