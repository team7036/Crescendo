package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants.Drivetrain.Swerve;

public class SwerveModule implements Sendable {

    // Create Variables for Motor and Encoder
    private final CANSparkMax m_driveMotor;
    private final RelativeEncoder m_driveEncoder;
    private final CANSparkMax m_turnMotor;
    private final CANcoder m_turnEncoder;

    // Create PID and Feed Forward Controllers
    private final PIDController m_drivePIDController = new PIDController(
        Swerve.Drive.PID.kP, 
        Swerve.Drive.PID.kI, 
        Swerve.Drive.PID.kD
    );
    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(
        Swerve.Drive.Feedforward.kS, 
        Swerve.Drive.Feedforward.kV
    );
    private final ProfiledPIDController m_turnPIDContoller = new ProfiledPIDController(
        Swerve.Turn.PID.kP, 
        Swerve.Turn.PID.kI, 
        Swerve.Turn.PID.kD,
        new TrapezoidProfile.Constraints( Swerve.Turn.MAX_VELOCITY, Swerve.Turn.MAX_ACCELERATION )
    );
    private final SimpleMotorFeedforward m_turnFeedForward = new SimpleMotorFeedforward(
        Swerve.Turn.Feedforward.kS, 
        Swerve.Turn.Feedforward.kV
    );

    public SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderID){

        m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turnPIDContoller.enableContinuousInput(-Math.PI, Math.PI);
        // Setup Turn Encoder
        m_turnEncoder = new CANcoder(turnEncoderID);
        // Setup Drive Encoder
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setPositionConversionFactor( Swerve.Drive.POSITION_CONVERSION ); // Convert from Rotations to meters
        m_driveEncoder.setVelocityConversionFactor( Swerve.Drive.VELOCITY_CONVERSION ); // Convert from RPM to m/s
    }

    public double getTurnPosition(){ // in rad
        return m_turnEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
    }

    public void setTurnPosition(double angle){ // in rad

        final double turnOutput = 
            m_turnPIDContoller.calculate(this.getTurnPosition(), angle);

        final double turnFeedFoward = m_turnFeedForward.calculate(m_turnPIDContoller.getSetpoint().velocity);

        m_turnMotor.setVoltage(turnOutput + turnFeedFoward);

    }

    public void setDriveVelocity(double velocity){ // in m/s

        final double driveOutput = 
            m_drivePIDController.calculate(m_driveEncoder.getVelocity(), velocity);

        final double driveFeedForward = 
            m_driveFeedForward.calculate( velocity );

        m_driveMotor.setVoltage(driveOutput + driveFeedForward);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(), new Rotation2d(this.getTurnPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(), new Rotation2d(this.getTurnPosition())
        );
    }

    public void setDesiredState( SwerveModuleState desiredState ){

        var rotation = new Rotation2d( this.getTurnPosition() );

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, rotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        state.speedMetersPerSecond *= state.angle.minus(rotation).getCos();

        this.setDriveVelocity(state.speedMetersPerSecond);
        this.setTurnPosition(state.angle.getRadians());

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("swervemodule");
        builder.addDoubleProperty("turn/position", this::getTurnPosition, null);
        builder.addDoubleProperty("drive/velocity", this.m_driveEncoder::getVelocity, null);
        builder.addDoubleProperty("drive/position", this.m_driveEncoder::getPosition, null);
    }

}
