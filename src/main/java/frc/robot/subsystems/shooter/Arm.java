package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.commands.ArmAngle;

public class Arm extends ProfiledPIDSubsystem {

    private final CANSparkMax motor;
    public final RelativeEncoder encoder;
    private final ArmFeedforward feedforward;
    
    public Arm() {
        super(
            new ProfiledPIDController(
            Constants.Shooter.Arm.PID.kP, 
            Constants.Shooter.Arm.PID.kI, 
            Constants.Shooter.Arm.PID.kD,
            new TrapezoidProfile.Constraints(
                Constants.Shooter.Arm.MAX_VELOCITY,
                Constants.Shooter.Arm.MAX_ACCELERATION
            ))
        );
        // Configure Motor
        motor = new CANSparkMax(Constants.Shooter.Ports.ARM_MOTOR, MotorType.kBrushless);
        motor.setSoftLimit(SoftLimitDirection.kReverse, Constants.Shooter.Arm.MIN_ANGLE);
        motor.setSoftLimit(SoftLimitDirection.kForward, Constants.Shooter.Arm.MAX_ANGLE);
        motor.setIdleMode(IdleMode.kBrake);
        // Configure Encoder
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(Constants.Shooter.Arm.POSITION_CONVERSION);
        encoder.setVelocityConversionFactor(Constants.Shooter.Arm.VELOCITY_CONVERSION);
        feedforward = new ArmFeedforward(
            Constants.Shooter.Arm.Feedforward.kS, 
            Constants.Shooter.Arm.Feedforward.kG, 
            Constants.Shooter.Arm.Feedforward.kV
        );
    }

    public void setAngle( double angle ){
        SmartDashboard.putNumber("desired angle", angle);
        double output = m_controller.calculate(encoder.getPosition(), angle);
        double feed = feedforward.calculate(angle, 0);
        motor.setVoltage(output + feed);
    }

    public void coast(){
        motor.setIdleMode(IdleMode.kCoast);
        disable();
    }


    @Override
    protected void useOutput(double output, State setpoint) {
        motor.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
    }

    @Override
    protected double getMeasurement() {
        return encoder.getPosition();
    }

    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("angle", this::getMeasurement, null);
        SmartDashboard.putData("set angle to 1", new ArmAngle(this, 1.0));
    }

}
