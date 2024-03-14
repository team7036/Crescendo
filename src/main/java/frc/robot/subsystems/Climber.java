package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber.Mode;

public class Climber extends SubsystemBase {

    private final CANSparkMax upMotor;
    private final CANSparkMax downMotor;
    private final double raiseSpeed = 0.15;
    private final double lowerSpeed = -0.75;
    private final RelativeEncoder upMotorEncoder;
    private final RelativeEncoder downMotorEncoder;

    public Mode mode = Mode.IDLE;

    public Climber() {
        // configure motor
        upMotor = new CANSparkMax(Constants.Climber.Ports.MOTOR_LIFT, MotorType.kBrushless);
        upMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.MIN_CLIMBER_HEIGHT) ;
        upMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.MAX_CLIMBER_HEIGHT);
        upMotor.setIdleMode(IdleMode.kBrake);
        upMotorEncoder = upMotor.getEncoder();

        downMotor = new CANSparkMax(Constants.Climber.Ports.MOTOR_LOWER, MotorType.kBrushless);
        downMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.MIN_CLIMBER_HEIGHT) ;
        downMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.MAX_CLIMBER_HEIGHT);
        downMotor.setIdleMode(IdleMode.kBrake);
        downMotorEncoder = downMotor.getEncoder();
    }

    public void up() {
        if(upMotorEncoder.getPosition() < Constants.Climber.MAX_CLIMBER_HEIGHT) {
            upMotor.set(raiseSpeed);
            downMotor.setIdleMode(IdleMode.kCoast);
        } else {
            stay();
        }
    }

    public void down() {
        if (downMotorEncoder.getPosition() > Constants.Climber.MIN_CLIMBER_HEIGHT) {
            downMotor.set(lowerSpeed);
            upMotor.setIdleMode(IdleMode.kCoast);
        }
        else {
            stay();
        }
    }

    public void stay(){
        downMotor.set(0);
        upMotor.set(0);
    }

    public void coast(){
        upMotor.setIdleMode(IdleMode.kCoast);
        downMotor.setIdleMode(IdleMode.kCoast);
    }

}
