package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.Mode;
import frc.robot.subsystems.Vision;

public class Shooter extends SubsystemBase {

    public Mode mode = Mode.IDLE;
    // Shooter Flywheels
    private final FlyWheels flyWheels;
    // Shooter Arm
    private final Arm arm;
    // Staging Servo
    private final Servo stagingServo;
    // Inputs
    private DigitalInput sensor;

    public Shooter(){
        // flywheels
        flyWheels = new FlyWheels();
        // Arm
        arm = new Arm();
        // Setup Staging Servo and Sensor
        stagingServo = new Servo(Constants.Shooter.Ports.STAGING_SERVO);
        sensor = new DigitalInput(Constants.Shooter.Ports.SENSOR);
    }

    /*
    public Command setAngleCommand( double angle ){
        return this.runOnce(() -> {
            arm.setGoal(angle);
            arm.enable();
        });
    }

    public Command setSpeedCommand( double rpm ){
        return this.runOnce(() -> flyWheels.setSpeed(rpm));
    }

    public Command idleCommand(){
        return this.runOnce( () -> flyWheels.idle() );
    }
    */
    @Override
    public void periodic(){
        
        // Intake mode intakes a note
        if ( mode == Mode.INTAKING ) {
            arm.setAngle(Constants.Shooter.Arm.INTAKE_ANGLE);
            flyWheels.setSpeed(Constants.Shooter.FlyWheels.INTAKE_SPEED);
            stagingServo.setAngle(180);
        // Manual Aiming is not used for now, but it will make the arm be controlled by the joysticks
        } else if ( mode == Mode.MANUAL_AIMING ) {
            // arm.setAngle(arm.encoder.getPosition() + (RobotContainer.operatorController.getLeftY() / 3)); 
            // flyWheels.setSpeed(0);
            // stagingServo.setAngle(90);
        // Speaker firing spins firing motors fast enough to launch the note to the speaker, and makes sure the angle aimed at the speaker
        } else if ( mode == Mode.SPEAKER_FIRING ) {
            arm.setAngle( Vision.calculateArmAngle() );
            flyWheels.setSpeed(Constants.Shooter.FlyWheels.SPEAKER_SPEED);
            if ( flyWheels.atSpeed(Constants.Shooter.FlyWheels.SPEAKER_SPEED) ){
                stagingServo.setAngle(0);
            } else {
                stagingServo.setAngle(90);
            }
            
        // Amp aim aims the arm at the amp
        } else if ( mode == Mode.AMP_AIM ){
            arm.setAngle( Constants.Shooter.Arm.AMP_ANGLE );
            flyWheels.setSpeed(0);
            stagingServo.setAngle(90);
            
        // amp firing spins firing motors fast enough to launch the note to the amp, and makes sure the angle aimed at the amp
        } else if ( mode == Mode.AMP_FIRING ){
            arm.setAngle( Constants.Shooter.Arm.AMP_ANGLE);
            flyWheels.setSpeed(Constants.Shooter.FlyWheels.AMP_SPEED);
            if ( flyWheels.atSpeed(Constants.Shooter.FlyWheels.AMP_SPEED) ){
                stagingServo.setAngle(0);
            } else {
                stagingServo.setAngle(90);
            }
        } else if (mode == Mode.SPEAKER_AIM) {
            arm.setAngle( Vision.calculateArmAngle() );
            flyWheels.setSpeed(0);
            stagingServo.setAngle(90);
        } else if (mode == Mode.SPEAKER_AIM_MANUAL) {
            arm.setAngle(Constants.Shooter.Arm.SPEAKER_ANGLE_SHORT);
            flyWheels.setSpeed(0);
            stagingServo.setAngle(90);
        } else if ( mode == Mode.TEST_IDLE ){
            arm.coast();
        }
        // test modes
        else if ( mode == Mode.TEST_INTAKING ){
            arm.coast();
            flyWheels.setSpeed(Constants.Shooter.FlyWheels.INTAKE_SPEED);
            stagingServo.setAngle(180);
        } else {
            arm.setAngle(Constants.Shooter.Arm.INTAKE_ANGLE);
            stagingServo.setAngle(90);
            flyWheels.setSpeed(0);
        // speaker aim aims the arm at the speaker
        }
    }

    @Override
    public void initSendable(SendableBuilder builder){
        // SmartDashboard.putData("shooter/pid", arm.getController());
        SmartDashboard.putData("arm",arm);
        SmartDashboard.putData("flywheels", flyWheels);
        builder.addDoubleProperty("speaker Z offset", Vision::getSpeakerZOffset, (offset)->Vision.setSpeakerZOffset(offset));
        builder.addDoubleProperty("speaker X offset", Vision::getSpeakerXOffset, (offset)->Vision.setSpeakerXOffset(offset));
        builder.addDoubleProperty("calculatedAngle", Vision::calculateArmAngle, null);      
    }



}
