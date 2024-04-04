package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AmpFire extends SequentialCommandGroup{
    public AmpFire() {
        addCommands(
            new ArmAngle(Constants.Shooter.Arm.AMP_ANGLE),
            new RunCommand(()->{
                RobotContainer.shooter.amp();
            })
        );
        addRequirements(RobotContainer.shooter);
    }
}
