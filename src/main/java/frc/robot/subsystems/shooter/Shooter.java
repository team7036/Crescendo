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
        setupDashboard();
        // flywheels
        flyWheels = new FlyWheels();
        // Arm
        arm = new Arm();
        // Setup Staging Servo and Sensor
        stagingServo = new Servo(Constants.Shooter.Ports.STAGING_SERVO);
        sensor = new DigitalInput(Constants.Shooter.Ports.SENSOR);
    }

    private void setupDashboard(){
        SmartDashboard.putData("Spinup Flywheels", this.setSpeedCommand(1000));
        SmartDashboard.putData("Set to Shooting Angle", this.setAngleCommand(1.5));
        SmartDashboard.putData("Set to Intake Angle", this.setAngleCommand(1.5));
        SmartDashboard.putData("shooter/arm", arm);
        SmartDashboard.putData("shooter/flywheels", flyWheels);
    }

    public Command setAngleCommand( double angle ){
        return this.run(()->arm.setAngle(angle))
            .until(arm.getController()::atSetpoint);
    }

    public Command setSpeedCommand( double rpm ){
        return this.run(() -> flyWheels.setSpeed(rpm))
            .until( () -> flyWheels.atSpeed(rpm) );
    }

    @Override
    public void periodic(){
        /*
                if ( mode == Mode.INTAKE ) {
            setAngle(0);
            motor.set(-0.25);
            stagingServo.setAngle(180);
        } else if ( mode == Mode.MANUAL_AIM ) {
            motor.set(0);
            stagingServo.setAngle(90);
        } else if ( mode == Mode.FIRE ) {
            setAngle(1.5);
            motor.set(1);
            if ( encoder.getVelocity() < 2500 ){
                stagingServo.setAngle(90);
            } else {
                stagingServo.setAngle(0);
            }
        } else {
            setAngle(0);
            motor.set(0);
            stagingServo.setAngle(90);
        }
         */
    }

    @Override
    public void initSendable(SendableBuilder builder){

    }



}
