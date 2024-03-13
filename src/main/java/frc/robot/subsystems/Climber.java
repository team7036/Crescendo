package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber.Mode;

public class Climber extends SubsystemBase {

    private final CANSparkMax raiseMotor;
    private final CANSparkMax liftMotor;

    public Mode mode = Mode.CLIMBER_DOWN;

    public Climber() {
        // configure motor
        raiseMotor = new CANSparkMax(Constants.Climber.Ports.MOTOR_LIFT, MotorType.kBrushless);
        raiseMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.MIN_CLIMBER_HEIGHT) ;
        raiseMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.MAX_CLIMBER_HEIGHT);
        raiseMotor.setIdleMode(IdleMode.kBrake);

        liftMotor = new CANSparkMax(Constants.Climber.Ports.MOTOR_LOWER, MotorType.kBrushless);
        liftMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.MIN_CLIMBER_HEIGHT) ;
        liftMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.MAX_CLIMBER_HEIGHT);
        liftMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setHeight( double height ) {
        // motor.set();
    }

    public void coast(){
        liftMotor.setIdleMode(IdleMode.kCoast);
        // disable();
    }

    public void periodic() {
        if( mode == Mode.CLIMBER_UP) {
            setHeight(Constants.Climber.MAX_CLIMBER_HEIGHT);
        } else if ( mode == Mode.CLIMBER_DOWN ) {
            setHeight( Constants.Climber.MIN_CLIMBER_HEIGHT );
        } else if ( mode == Mode.COAST ) {
            coast();
        }
    }
}
