package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Shooter.Mode;
import frc.robot.subsystems.Climber;
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
    private final Climber climber = new Climber();

    public RobotContainer(){
        SmartDashboard.putData("drivetrain", swerve);
        SmartDashboard.putData("shooter", shooter);
        SmartDashboard.putData("intake", intake);
    }

    // controls the operator controller

    public void operate() {

        if ( operatorController.getHID().getLeftBumper() ){ // Left Bumper aims at the speaker
            if ( operatorController.getHID().getRightBumper() ){ // Right bumper fires
                shooter.mode = Mode.SPEAKER_FIRING;
            } else {
                shooter.mode = Mode.SPEAKER_AIM;
            }
        } else if ( operatorController.getHID().getAButton() ){ // A aims at the Amp
            if ( operatorController.getHID().getRightBumper() ){ // Right Bumper Fires
                shooter.mode = Mode.AMP_FIRING;
            } else {
                shooter.mode = Mode.AMP_AIM;
            }
        } else {
            if ( (driverController.getHID().getLeftBumper() || operatorController.getHID().getYButton()) && !intake.isLoaded()) { // left bumper intakes
                shooter.mode = Mode.INTAKING;
                intake.run();
            }  else {
                shooter.mode = Mode.IDLE;
                intake.stop();  
            }
        }

        if ( operatorController.getHID().getLeftTriggerAxis() > 0.25 ) { // RT -> climber up
            climber.up();
        } 
        else if ( operatorController.getHID().getRightTriggerAxis() > 0.25 ) { // LT -> climber down
            climber.down();
        } else {
            climber.stay();
        }

    }

    // controls the driver controller

    public void drive( double period ){

        boolean slow = driverController.getHID().getRightBumper() ? true : false;

        final var x =
            m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getLeftY(), 0.02))
                * (slow ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED);

        final var y =
            -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getLeftX(), 0.02))
                * (slow ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED);

        final var rot =
            -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getRightX(), 0.02))
                * (slow ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED);

        swerve.drive(x, y, rot, period);
    }
    
}
