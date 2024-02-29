package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(Constants.Controllers.DRIVER);
    //private final XboxController driverController = new XboxController(Constants.Controllers.DRIVER);
    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_ySpeedlimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private final Drivetrain swerve = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    public RobotContainer(){
        SmartDashboard.putData("drivetrain", swerve);
        SmartDashboard.putData("shooter", shooter);
    }

    public void teleopDrive(){

        final var xSpeed =
        -m_xSpeedLimiter.calculate(MathUtil.applyDeadband(-driverController.getLeftY(), 0.05));

        final var ySpeed =
        -m_ySpeedlimiter.calculate(MathUtil.applyDeadband(driverController.getLeftX(), 0.05));

        final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX(), 0.05));

        swerve.drive(xSpeed, ySpeed, rot, false);   
    }

}
