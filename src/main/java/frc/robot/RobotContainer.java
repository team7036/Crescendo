package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Shooter.Mode;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.shooter.ArmAngle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {

    // Initialize Controllers
    private final CommandXboxController driverController = new CommandXboxController(Constants.Controllers.DRIVER);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.Controllers.OPERATOR);

    // Initialize Subsystems
    public static Drivetrain swerve = new Drivetrain();
    public static Shooter shooter = new Shooter();
    public static Intake intake = new Intake();
    public static Climber climber = new Climber();
    public static boolean fieldRelative = false;

    public RobotContainer(){

        SmartDashboard.putData("drivetrain", swerve);
        SmartDashboard.putData("shooter", shooter);
        SmartDashboard.putData("intake", intake);
        setDriverBindings();
        setOperatorBindings();
    }

    private void setOperatorBindings(){
        /*
        operatorController.leftBumper()
            .onTrue( new SetArmAngle(1) )
            .onFalse( new SetArmAngle(0) );
            */

        operatorController.a()
            .onTrue( new ArmAngle(Constants.Shooter.Arm.AMP_ANGLE) )
            .onFalse( new ArmAngle(0) );
    }

    private void setDriverBindings(){

        swerve.setDefaultCommand(
            new RunCommand(
                () -> swerve.drive( driverController.getLeftY() , -driverController.getLeftX(), -driverController.getRightX()),
                swerve
            )
        );

        driverController.leftBumper()
            .onTrue( new SequentialCommandGroup(
                new ArmAngle(Constants.Shooter.Arm.INTAKE_ANGLE),
                new RunCommand(()->{
                    intake.run();
                    shooter.intake();
                }).until( RobotContainer.intake::isLoaded )
            )).onFalse(new InstantCommand(()->{
                intake.stop();
                shooter.stop();
            }));
    }

    // controls the operator controller

    public void operate() {
        /*
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
         */


    }

    // controls the driver controller

    public void resetGyro(){
        swerve.resetGyro();
    }

    /*
    public void drive( double period ){

        boolean slow = driverController.getHID().getRightBumper() ? true : false;

        SmartDashboard.putBoolean("slowMode", slow);
        if ( driverController.getHID().getAButtonPressed() ){
            fieldRelative = !fieldRelative;
        }

        SmartDashboard.putBoolean("FieldRelative", fieldRelative);

        final var x =
            m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getHID().getLeftY(), 0.02))
                * (slow ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED);

        final var y =
            m_yspeedLimiter.calculate(MathUtil.applyDeadband(-driverController.getHID().getLeftX(), 0.02))
                * (slow ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED);

        final var rot =
            m_rotLimiter.calculate(MathUtil.applyDeadband(-driverController.getHID().getRightX(), 0.02))
                * (slow ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED);

        swerve.drive(x, y, rot);
    }
    */
}
