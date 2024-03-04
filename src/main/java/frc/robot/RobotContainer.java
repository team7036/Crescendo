package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Shooter.Mode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(Constants.Controllers.DRIVER);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private final Drivetrain swerve = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();

    public RobotContainer(){
        //SmartDashboard.putData("drivetrain", swerve);
        SmartDashboard.putData("shooter", shooter);
    }

    public void testArmPID(){

    }

    public void testDrive(){

        if ( driverController.a().getAsBoolean()) {
            shooter.mode = Mode.MANUAL_AIMING;
            if ( driverController.rightBumper().getAsBoolean() ){
                shooter.mode = Mode.FIRING;
            }
        } else if( driverController.y().getAsBoolean() ){
            shooter.mode = Mode.AMP_AIM;
            if ( driverController.rightBumper().getAsBoolean() ){
                shooter.mode = Mode.AMP_SCORE;
            }

        } else if ( driverController.leftBumper().getAsBoolean() ){
            shooter.mode = Mode.INTAKING;
            intake.run();
        } else {
            shooter.mode = Mode.IDLE;
            intake.stop();
        }

        final var x =
            m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftY(), 0.02))
                * Constants.Drivetrain.MAX_DRIVE_SPEED;

        final var y =
            -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftX(), 0.02))
                * Constants.Drivetrain.MAX_DRIVE_SPEED;

        final var rot =
            -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX(), 0.02))
                * Constants.Drivetrain.MAX_DRIVE_SPEED;

        swerve.drive(x, y, rot);
        
    }

}
