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
    private final XboxController driverControllerToRumble = new XboxController(Constants.Controllers.DRIVER);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private final Drivetrain swerve = new Drivetrain();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();

    boolean vibrate;

    public RobotContainer(){
        SmartDashboard.putData("drivetrain", swerve);
        SmartDashboard.putData("shooter", shooter);
        SmartDashboard.putData("intake", intake);
        vibrate = false;
    }

    // controls the Operator controller
    public void operate() {

        if ( operatorController.getHID().getLeftBumper() ){ // Left Bumper aims at the speaker
            if ( operatorController.getHID().getRightBumper() ){ // Right bumper fires
                shooter.mode = Mode.SPEAKER_FIRING;
                vibrate = true; 
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
            if ( driverController.getHID().getLeftBumper() && !intake.isLoaded()) {
                shooter.mode = Mode.INTAKING;
                vibrate = true;
                intake.run();
            } else if ( driverController.getHID().getRightTriggerAxis() > 0.25 ) {
                climber.mode = Constants.Climber.Mode.CLIMBER_UP;
            } else {
                shooter.mode = Mode.IDLE;
                vibrate = false;
                intake.stop();  
            }
            
        }

        if ( vibrate ) {
            driverControllerToRumble.setRumble(RumbleType.kLeftRumble, 1);
            driverControllerToRumble.setRumble(RumbleType.kRightRumble, 1);
        } else {
            driverControllerToRumble.setRumble(RumbleType.kLeftRumble, 0);
            driverControllerToRumble.setRumble(RumbleType.kRightRumble, 0);
        }

    }

    public void drive(){
        // handles drive speed and slow mode
        double driveMultiplier;

        if ( driverController.getHID().getRightBumper() ) {
            driveMultiplier = Constants.Drivetrain.SLOW_DRIVE_SPEED;
        } else {
            driveMultiplier = 1;
        }

        final var x =
            m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getLeftY(), 0.02))
                * driveMultiplier;

        final var y =
            -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getLeftX(), 0.02))
                * driveMultiplier;

        final var rot =
            -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getRightX(), 0.02))
                * driveMultiplier;

        swerve.drive(x, y, rot);
    }
    
}
