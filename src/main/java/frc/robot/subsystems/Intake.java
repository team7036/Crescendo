package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    CANSparkMax motor;
    Ultrasonic intakeSensor;
    static DigitalInput loadedSensor;

    public Intake(){
        motor = new CANSparkMax( Constants.Intake.Ports.MOTOR , MotorType.kBrushless);
        intakeSensor = new Ultrasonic( Constants.Intake.Ports.INTAKE_SENSOR_INPUT, Constants.Intake.Ports.INTAKE_SENSOR_INPUT);
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

    public static boolean isLoaded(){
        return !loadedSensor.get();
    }

    public void run() {
        // pull in note
        motor.set(-1);
    }

    public void stop(){
        motor.set(0);
    }

}
