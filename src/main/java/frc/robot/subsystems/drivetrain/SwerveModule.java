package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    // Establish Swerve Module Constants
    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI;
    private static final double kWheelDiameter = 0.102; // in meters
    private static final double kDriveGearRatio = 6.75;

    // Create Variables for Motor and Encoder
    private final CANSparkMax m_driveMotor;
    private final RelativeEncoder m_driveEncoder;
    private final CANSparkMax m_turnMotor;
    private final CANcoder m_turnEncoder;
    private final CANcoderConfiguration m_turnEncoderConfig = new CANcoderConfiguration();

    // Create PID Controllers and set Gains
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    private final TrapezoidProfile.Constraints turningConstraints = new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration);
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
        m_driveEncoder.setPositionConversionFactor( Math.PI * kWheelDiameter / kDriveGearRatio );
        m_driveEncoder.setVelocityConversionFactor( Math.PI * kWheelDiameter / kDriveGearRatio );
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(), new Rotation2d(m_turnEncoder.getPosition().getValue())
        );
    }

    // returns current position of the module
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(), new Rotation2d(m_turnEncoder.getPosition().getValue())
        );
    }

    public void setDesiredState( SwerveModuleState desiredState ){
        var encoderRotation = new Rotation2d(m_turnEncoder.getPosition().getValue());

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        // direction of travel that can occur when modules change directions. This results in smoother
        // driving.
        state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = 
            m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedForward = 
            m_driveFeedForward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = 
            m_turnPIDContoller.calculate(m_turnEncoder.getPosition().getValue(), state.angle.getRadians());

        final double turnFeedFoward = 
            m_turnFeedForward.calculate(m_turnPIDContoller.getSetpoint().velocity);

        m_driveMotor.setVoltage(driveOutput + driveFeedForward);
        m_turnMotor.setVoltage(turnOutput + turnFeedFoward);
    }

}
