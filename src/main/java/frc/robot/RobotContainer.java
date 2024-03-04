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
    private final CommandXboxController operatorController = new CommandXboxController(Constants.Controllers.OPERATOR);
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

    // Controls both Driver controller
    public void teleopDrive(){
        if (driverController.a().getAsBoolean() ){
            shooter.mode = Mode.INTAKING;
            intake.run();
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

    // controls the Operator controller
    public void teleopOp() {
        // A is preset amp angles
        if (operatorController.a().getAsBoolean()) {
            shooter.mode = Mode.AMP_AIM;
        // X sets the velocity for the motors when aiming at the amp
        } if (operatorController.x().getAsBoolean()) {
            shooter.mode = Mode.AMP_FIRING;
        }

        // B is the preset Speaker angle
        if(operatorController.b().getAsBoolean() ){
            shooter.mode = Mode.SPEAKER_AIM;
        // Y sets the velocity for the motors when aiming at the speaker
        } if (operatorController.y().getAsBoolean() ){
            shooter.mode = Mode.SPEAKER_FIRING;
        } else {
            shooter.mode = Mode.IDLE;
            intake.stop();
        }
        // RT drops the note into the firing mechanism, shooting the note, but only if the shooter is already in a firing mode
        if (operatorController.rightTrigger().getAsBoolean() && shooter.mode == Mode.AMP_FIRING || shooter.mode == Mode.SPEAKER_FIRING){
            shooter.mode = Mode.SCORE;
        }
    }

    public void testDrive(){
        teleopDrive();
        teleopOp();
    }

}
