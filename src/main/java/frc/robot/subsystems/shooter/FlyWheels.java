package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlyWheels extends SubsystemBase {
        // Motors
    private final CANSparkMax motor;
    private final CANSparkMax bottomMotor;
    private final SimpleMotorFeedforward feedForward;
    // Encoder
    private final RelativeEncoder encoder;

    public FlyWheels(){
        motor = new CANSparkMax(Constants.Shooter.Ports.TOP_MOTOR, MotorType.kBrushless);
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kCoast);
        bottomMotor = new CANSparkMax(Constants.Shooter.Ports.BOTTOM_MOTOR, MotorType.kBrushless);
        bottomMotor.follow(motor);
        bottomMotor.setIdleMode(IdleMode.kCoast);
        feedForward = new SimpleMotorFeedforward(1, 0.0048);
        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(Constants.Shooter.FlyWheels.SPEED_CONVERSION);
    }

    public void idle(){
        motor.setVoltage(0);
    }

    public void setSpeed( double rpm ){
        motor.setVoltage( feedForward.calculate( rpm ) );
    }

    public boolean atSpeed( double rpm ){
        return encoder.getVelocity() >= rpm;
    }

    public void initSendable( SendableBuilder builder ){
        builder.addDoubleProperty("speedRPM", this.encoder::getVelocity, null);
    }

}
