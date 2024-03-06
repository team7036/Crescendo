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
        
        if (driverController.getHID().getAButton() ){
            shooter.mode = Mode.INTAKING;
            intake.run();
        } else {
            intake.stop();
        } 
    
        // if ( intake.seesNote() && !intake.isLoaded() ) {
        //     shooter.mode = Mode.INTAKING;
        //     intake.run();
        // }


        final var x =
            m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getLeftY(), 0.02))
                * Constants.Drivetrain.MAX_DRIVE_SPEED;

        final var y =
            -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getLeftX(), 0.02))
                * Constants.Drivetrain.MAX_DRIVE_SPEED;

        final var rot =
            -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getRightX(), 0.02))
                * Constants.Drivetrain.MAX_DRIVE_SPEED;

        swerve.drive(x, y, rot);
    }

    // controls the Operator controller
    public void teleopOp() {
        // if no button is pressed, set the shooter mode to idle
        if(!operatorController.getHID().getAButton() &&
        !operatorController.getHID().getXButton() && 
        !operatorController.getHID().getBButton() && 
        !operatorController.getHID().getYButton() &&
        // operatorController.getLeftY() == 0 && 
        !operatorController.getHID().getRightBumper() &&
        !driverController.getHID().getAButton()) {
            // if nothing is pressed, the robot is idle
            shooter.mode = Mode.IDLE;
        } else if (operatorController.getHID().getYButton()) {
            // y sets the shooting motor velocity for speaker 
            shooter.mode = Mode.SPEAKER_FIRING;
        } else if (operatorController.getHID().getXButton()) {
             // X sets the velocity for the motors when aiming at the amp
            shooter.mode = Mode.AMP_FIRING;
        } else if (operatorController.getHID().getBButton()) {
            // B sets the angle for the arm when aiming at the speaker
            shooter.mode = Mode.SPEAKER_AIM;
        } else if (operatorController.getHID().getAButton()) {
            // A sets the angle for the arm when aiming at the amp
            shooter.mode = Mode.AMP_AIM;
        } 
        // Check (at the highest priority) if the trigger is pressed and the robot is ready to fire. Then drop the note into the firing mechanism
        if (
            operatorController.getHID().getRightBumper() && 
            ( shooter.mode == Mode.AMP_FIRING || shooter.mode == Mode.SPEAKER_FIRING )
        ) {
            // RT releases the note into the motors given that they are running 
            shooter.mode = Mode.SCORE;
        }
    }

    public void testDrive(){
        //teleopOp();
        //teleopDrive();
    }

    public void testShooter(){

        if (driverController.getHID().getAButton()) {
            shooter.mode = Mode.TEST_INTAKING;
        } else if (operatorController.getHID().getYButton()) {
            shooter.mode = Mode.TEST_REV;
        } else if (operatorController.getHID().getRightBumper()) {
            shooter.mode = Mode.TEST_FIRE;
        } else {
            shooter.mode = Mode.TEST_IDLE;
        }
        // closest one: 1.538 -> 1 m
        // farthest one: 1.2801 -> 3.3528 m
    }

}
