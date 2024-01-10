package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {
    // motors
    // import xbox controller

    private double maxMotorSpeed = 0.0;
    private final CANSparkMax m_shooterLeft;
    private final CANSparkMax m_shooterRight;

    public Shooter(int leftMotorID, int rightMotorID){
        m_shooterLeft = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        m_shooterRight = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    }

    public boolean isLoaded(){
        return false;
    }

    public void setSpeed( double speed ) {
        m_shooterLeft.set( speed );
        m_shooterRight.set( -speed );
    }

    public double getSpeed() {
        return 0.0;
    }
}
