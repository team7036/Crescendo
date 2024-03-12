package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final CANSparkMax motor;
    private final Ultrasonic intakeSensor;
    private static DigitalInput loadedSensor;

    public Intake(){
        motor = new CANSparkMax( Constants.Intake.Ports.MOTOR , MotorType.kBrushless);
        intakeSensor = new Ultrasonic( Constants.Intake.Ports.INTAKE_SENSOR_INPUT, Constants.Intake.Ports.INTAKE_SENSOR_OUTPUT);
        loadedSensor = new DigitalInput( Constants.Intake.Ports.LOADED_SENSOR );
    }

    public boolean seesNote(){
        double intakeDistanceMM = intakeSensor.getRangeMM();
        double actuatingDistance = 50;
        if ( intakeDistanceMM < actuatingDistance) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isLoaded(){
        return !loadedSensor.get();
    }

    public void run() {
        // pull in note
        if ( isLoaded() ){
            motor.set(0);
        } else {
            motor.set(Constants.Intake.INTAKE_SPEED);
        }
    }

    public void stop(){
        // stop running
        motor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addBooleanProperty("loaded", this::isLoaded, null);        
    }


}
