package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.Mode;

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

    @Override
    public void periodic(){

        if ( mode == Mode.INTAKING ) {
            arm.setAngle(0);
            flyWheels.setSpeed(-10);
            stagingServo.setAngle(180);
        } else if ( mode == Mode.MANUAL_AIMING ) {
            arm.setAngle(1.5); //TODO: Change to controlled by joystick
            flyWheels.setSpeed(0);
            stagingServo.setAngle(90);
        } else if ( mode == Mode.FIRING ) {
            arm.setAngle(1.5); // TODO
            flyWheels.setSpeed(2500);
            if ( flyWheels.atSpeed(2500) ){
                stagingServo.setAngle(0);
            } else {
                stagingServo.setAngle(90);
            }
        } else if ( mode == Mode.AMP_AIM ){
            arm.setAngle( Constants.Shooter.Arm.AMP_ANGLE );
            flyWheels.setSpeed(0);
            stagingServo.setAngle(90);
        } else if ( mode == Mode.AMP_SCORE ){
            arm.setAngle(Constants.Shooter.Arm.AMP_ANGLE);
            flyWheels.setSpeed(50);
            stagingServo.setAngle(0);
        } else if (mode == Mode.SPEAKER_AIM) {
            arm.setAngle( Constants.Shooter.Arm.SPEAKER_ANGLE );
            flyWheels.setSpeed(0);
            stagingServo.setAngle(90);
        } else if (mode == Mode.SPEAKER_SCORE) {
            arm.setAngle(Constants.Shooter.Arm.SPEAKER_ANGLE);
            flyWheels.setSpeed(200);
            stagingServo.setAngle(0);
        }
        else {
            arm.setAngle(0);
            flyWheels.setSpeed(0);
            stagingServo.setAngle(90);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder){
        SmartDashboard.putData("shooter/armTo0", setAngleCommand(0));
        SmartDashboard.putData("shooter/armTo1", setAngleCommand(1));
        SmartDashboard.putData("shooter/pid", arm.getController());
        builder.addDoubleProperty("angle", arm::getMeasurement, null);
    }



}
