package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    CANSparkMax motor;
    DigitalInput sensor;

    public Intake(){
        motor = new CANSparkMax( Constants.Intake.Ports.MOTOR , MotorType.kBrushless);
        sensor = new DigitalInput( Constants.Intake.Ports.SENSOR );
    }

    public boolean isLoaded(){
        // Use a sensor to see if we have a Note currently on the robot
        return true;
    }

    public void run() {
        // pull in note
        motor.set(-1);
    }

    public void stop(){
        motor.set(0);
    }

}
