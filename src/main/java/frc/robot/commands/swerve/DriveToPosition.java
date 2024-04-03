package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPosition extends Command{

    private final ProfiledPIDController rotController = new ProfiledPIDController(0, 0, 0, null);
    private final PIDController xController = new PIDController(0, 0, 0);
    private final PIDController yController = new PIDController(0, 0, 0);

    public DriveToPosition ( double xPos, double yPos, double rot ) {

    }

    @Override
    public void initialize() {

    }

}
