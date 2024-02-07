package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Drivetrain {

    // Establish Drivetrain Constants
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;

    // Establish Drivetrain Geometry
    /*
     * 
     */
    private final Translation2d m_frontLeftLocation = new Translation2d(-0.63 / 2, 0.63 / 2);
    private final Translation2d m_frontRightLocation = new Translation2d(0.63 / 2, 0.63 / 2);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.63 / 2, -0.63 / 2);
    private final Translation2d m_backRightLocation = new Translation2d(0.63 / 2, -0.63 / 2);

    // Create Swerve Module Objects
    private final SwerveModule m_frontLeft = new SwerveModule(23, 22, 30);
    private final SwerveModule m_frontRight = new SwerveModule(17, 16, 31);
    private final SwerveModule m_backLeft = new SwerveModule(21, 20, 32);
    private final SwerveModule m_backRight = new SwerveModule(19, 18, 33);

    // Setup NavX Gyro
    private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);

    // Establish Variables for Kinematics and Odemetry
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    public Drivetrain() {
        m_gyro.reset();
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot), periodSeconds));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        m_odometry.update(m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }
}
