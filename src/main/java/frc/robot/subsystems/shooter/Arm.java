package frc.robot.subsystems.shooter;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class Arm extends ProfiledPIDSubsystem {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
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
        // Set Default Command
        m_controller.setTolerance(0.05, 0.05);
    }

    public void coast(){
        motor.setIdleMode(IdleMode.kCoast);
        disable();
    }

    public Command setAngleCommand( double angle ){
        return this.runOnce(()->{
            this.setGoal(angle);
            this.enable();
        })
            .until(m_controller::atGoal)
            .finallyDo(()->{
                this.setGoal(Constants.Shooter.Arm.INTAKE_ANGLE);
                this.enable();
            })
            .withName("Set Angle to "+angle);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        motor.setVoltage(output + feedforward.calculate(setpoint.position, 0));
    }

    @Override
    protected double getMeasurement() {
        return encoder.getPosition();
    }

    public void initSendable(SendableBuilder builder){
        super.initSendable(builder);
        builder.addDoubleProperty("angle", this::getMeasurement, null);
        builder.addBooleanProperty("At Goal", this.m_controller::atGoal, null);
        SmartDashboard.putData("shooter/arm/pid",m_controller);
    }

}
