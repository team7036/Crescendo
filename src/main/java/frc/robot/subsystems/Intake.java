package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake {

    CANSparkMax m_intakeMotor;

    public Intake(int intakeMotorCANID){
        m_intakeMotor = new CANSparkMax(intakeMotorCANID, MotorType.kBrushless);
    }

    public boolean isLoaded(){
        // Use a sensor to see if we have a Note currently on the robot
        return true;
    }

    public void run() {
        // pull in note
        m_intakeMotor.set(-0.5);
    }

    public void stop(){
        m_intakeMotor.set(0);
    }

}
