package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class SysIdRoutineBot {
    private final Drivetrain swerve = new Drivetrain();
    private final Shooter shooter = new Shooter();
    CommandXboxController driveController = new CommandXboxController(0);
}
