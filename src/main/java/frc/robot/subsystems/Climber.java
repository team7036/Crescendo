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

    private final CANSparkMax topMotor;
    private final CANSparkMax bottomMotor;
    private final double raiseSpeed = 0.15;
    private final double lowerSpeed = -0.75;
    private final RelativeEncoder topMotorEncoder;
    private final RelativeEncoder bottomMotorEncoder;

    public Mode mode = Mode.IDLE;

    public Climber() {
        // configure motor
        topMotor = new CANSparkMax(Constants.Climber.Ports.TOP_MOTOR, MotorType.kBrushless);
        topMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.TOP_REVERSE_LIMIT) ;
        topMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.TOP_FORWARD_LIMIT);
        topMotor.setIdleMode(IdleMode.kBrake);
        topMotorEncoder = topMotor.getEncoder();

        bottomMotor = new CANSparkMax(Constants.Climber.Ports.BOTTOM_MOTOR, MotorType.kBrushless);
        bottomMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Climber.BOTTOM_REVERSE_LIMIT) ;
        bottomMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.Climber.BOTTOM_FORWARD_LIMIT);
        bottomMotor.setIdleMode(IdleMode.kBrake);
        bottomMotorEncoder = bottomMotor.getEncoder();
    }

    public void up() {
        topMotor.set(Constants.Climber.TOP_UP_SPEED);
        bottomMotor.set(Constants.Climber.BOTTOM_UP_SPEED);
    }

    public void down() {
        bottomMotor.set(Constants.Climber.BOTTOM_DOWN_SPEED);
        topMotor.setIdleMode(IdleMode.kCoast);
    }

    public void stay(){
        topMotor.setIdleMode(IdleMode.kBrake);
        bottomMotor.setIdleMode(IdleMode.kBrake);
        bottomMotor.set(0);
        topMotor.set(0);
    }

    public void coast(){
        topMotor.setIdleMode(IdleMode.kCoast);
        bottomMotor.setIdleMode(IdleMode.kCoast);
    }

}
