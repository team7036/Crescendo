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

    // Create PID Controllers and set Gains
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    private final TrapezoidProfile.Constraints turningConstraints = new TrapezoidProfile.Constraints(Swerve.MAX_ANGULAR_VELOCITY, Swerve.MAX_ANGULAR_ACCELERATION);
    private final ProfiledPIDController m_turnPIDContoller = new ProfiledPIDController(1, 0, 0, turningConstraints);

    // Setup Motor Feeds
    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedForward = new SimpleMotorFeedforward(1, 0.5);

    public SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderID){

        m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turnPIDContoller.enableContinuousInput(-Math.PI, Math.PI);
        // Setup Turn Encoder
        m_turnEncoder = new CANcoder(turnEncoderID);
        // Setup Drive Encoder
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setPositionConversionFactor( Swerve.POSITION_CONVERSION_FACTOR ); // Convert from Rotations to meters
        m_driveEncoder.setVelocityConversionFactor( Swerve.VELOCITY_CONVERSION_FACTOR ); // Convert from RPM to m/s
    }

    public double getTurnPosition(){
        return m_turnEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
    }

    public void setTurnPosition(double angle){ // in rad

        final double turnOutput = 
            m_turnPIDContoller.calculate(this.getTurnPosition(), angle);

        final double turnFeedFoward = 
            m_turnFeedForward.calculate(m_turnPIDContoller.getSetpoint().velocity);

        m_turnMotor.setVoltage(turnOutput + turnFeedFoward);

    }

    public double getDrivePosition(){
        return m_driveEncoder.getPosition();
    }

    public double getDriveVelocity(){ // in m/s
        return m_driveEncoder.getVelocity();
    }

    public void setDriveVelocity(double velocity){ // in m/s

        final double driveOutput = 
            m_drivePIDController.calculate(this.getDriveVelocity(), velocity);

        final double driveFeedForward = 
            m_driveFeedForward.calculate(velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedForward);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            this.getDriveVelocity(), new Rotation2d(this.getTurnPosition())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            this.getDrivePosition(), new Rotation2d(this.getTurnPosition())
        );
    }

    public void setDesiredState( SwerveModuleState desiredState ){

        var encoderRotation = new Rotation2d( this.getTurnPosition() );

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        this.setDriveVelocity(state.speedMetersPerSecond);
        this.setTurnPosition(state.angle.getRadians());

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        builder.setSmartDashboardType("swervemodule");
        builder.addDoubleProperty("turn/position", this::getTurnPosition, this::setTurnPosition);
        builder.addDoubleProperty("drive/velocity", this::getDriveVelocity, this::setDriveVelocity);
        builder.addDoubleProperty("drive/position", this::getDrivePosition, null);
    }

}
