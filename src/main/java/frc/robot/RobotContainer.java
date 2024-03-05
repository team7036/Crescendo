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
        } else if(!driverController.a().getAsBoolean()){
            shooter.mode = Mode.IDLE;
        } 
    
        // if ( intake.seesNote() && !intake.isLoaded() ) {
        //     shooter.mode = Mode.INTAKING;
        //     intake.run();
        // }


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
        // if no button is pressed, set the shooter mode to idle
        if(!operatorController.a().getAsBoolean() &&
        !operatorController.x().getAsBoolean() && 
        !operatorController.b().getAsBoolean() && 
        !operatorController.y().getAsBoolean() &&
        operatorController.getLeftY() == 0 && 
        !operatorController.rightTrigger().getAsBoolean()) {
            // if nothing is pressed, the robot is idle
            shooter.mode = Mode.IDLE;
        } else if (operatorController.y().getAsBoolean()) {
            // y sets the shooting motor velocity for speaker 
            shooter.mode = Mode.SPEAKER_FIRING;
        } else if (operatorController.x().getAsBoolean()) {
             // X sets the velocity for the motors when aiming at the amp
            shooter.mode = Mode.AMP_FIRING;
        } else if (operatorController.b().getAsBoolean()) {
            // B sets the angle for the arm when aiming at the speaker
            shooter.mode = Mode.SPEAKER_AIM;
        } else if (operatorController.a().getAsBoolean()) {
            // A sets the angle for the arm when aiming at the amp
            shooter.mode = Mode.AMP_AIM;
        } 
        // Check (at the highest priority) if the trigger is pressed and the robot is ready to fire. Then drop the note into the firing mechanism
        if (
            operatorController.rightTrigger().getAsBoolean() && 
            ( shooter.mode == Mode.AMP_FIRING || shooter.mode == Mode.SPEAKER_FIRING )
        ) {
            // RT releases the note into the motors given that they are running 
            shooter.mode = Mode.SCORE;
        }
    }

    public void testDrive(){
        teleopDrive();
        teleopOp();
        // change
    }

}
