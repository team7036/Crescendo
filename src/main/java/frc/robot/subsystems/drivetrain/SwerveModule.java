package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class SwerveModule {

    // Establish Swerve Module Constants
    private static final double kWheelRadius = 0.058;
    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; 

    // Create Variables for Motor and Encoder
    private final PWMSparkMax m_driveMotor;
    private final PWMSparkMax m_turnMotor;
    private final CANcoder m_turnEncoder;

    // Create PID Controllers and set Gains
    private final TrapezoidProfile.Constraints turningConstraints = new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration);
    private final ProfiledPIDController m_turnPIDContoller = new ProfiledPIDController(1, 0, 0, turningConstraints);
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // Setup Motor Feeds
    private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedForward = new SimpleMotorFeedforward(1, 0.5);


    public SwerveModule(int turnMotorID, int driveMotorID, int turnEncoderID){
        m_turnMotor = new PWMSparkMax(turnMotorID);
        m_driveMotor = new PWMSparkMax(driveMotorID);
        m_turnPIDContoller.enableContinuousInput(-Math.PI, Math.PI);
        m_turnEncoder = new CANcoder(turnEncoderID);
    }

}
