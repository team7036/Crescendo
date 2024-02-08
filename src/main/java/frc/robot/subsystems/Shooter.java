package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Shooter {

    // Constants
    private double maxShooterAcceleration = 1.0; // TODO
    private double maxAngleVelocity = 1.0; // TODO

    // Shooter Motors
    private double maxMotorSpeed = 0.0; // TODO
    private final CANSparkMax m_top;
    private final CANSparkMax m_bottom;
    private final RelativeEncoder m_shooterEncoder;
    private final PIDController m_shooterPID = new PIDController(1, 0, 0); // TODO
    private final SimpleMotorFeedforward m_shooterFeedForward = new SimpleMotorFeedforward(2, 1); // TODO
    // Angle Motor
    private final CANSparkMax m_angle;
    private final RelativeEncoder m_angleEncoder;
    private final PIDController m_anglePID = new PIDController(1, 0, 0);
    private final ArmFeedforward m_angleFeedforward = new ArmFeedforward(1, 0, 0);


    public Shooter(int topMotorID, int bottomMotorID, int angleMotorID){
        // Setup Shooting Motors
        m_top = new CANSparkMax(topMotorID, MotorType.kBrushless);
        m_top.setIdleMode(IdleMode.kCoast);
        m_bottom = new CANSparkMax(topMotorID, MotorType.kBrushless);
        m_bottom.setIdleMode(IdleMode.kCoast);
        m_bottom.setInverted(true);
        m_bottom.follow(m_top);
        m_shooterEncoder = m_top.getEncoder();
        m_shooterEncoder.setVelocityConversionFactor(1); // TODO
        // Setup Angle Motor
        m_angle = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        m_angleEncoder = m_angle.getEncoder();
        m_angleEncoder.setPositionConversionFactor(1); // TODO
    }

    public boolean isLoaded(){
        return false; // TODO
    }

    public void setSpeed( double speedRPM ) { // in RPM
        m_top.setVoltage( m_shooterPID.calculate( this.getSpeed() , speedRPM) + m_shooterFeedForward.calculate(speedRPM, maxShooterAcceleration) );
    }

    public double getSpeed() { // in RPM
        return m_shooterEncoder.getVelocity();
    }

    public void setAngle( double angle ) { // in rads
        m_angle.setVoltage( m_anglePID.calculate( this.getAngle(), angle ) + m_angleFeedforward.calculate(angle, maxAngleVelocity) );
    }

    public double getAngle() { // in rads
        return m_angleEncoder.getPosition();
    }

    public void aim(){
        double[] pose = Vision.AprilTag.getBotPose();
    }
}
