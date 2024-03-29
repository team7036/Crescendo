package frc.robot.subsystems.drivetrain;
import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain.*;


public class Drivetrain extends SubsystemBase {

    double targetHeading;

    // Create Swerve Module Objects
    private final SwerveModule m_frontLeft = new SwerveModule(Ports.FL_TURN, Ports.FL_DRIVE, Ports.FL_ENCODER);
    private final SwerveModule m_frontRight = new SwerveModule(Ports.FR_TURN, Ports.FR_DRIVE, Ports.FR_ENCODER);
    private final SwerveModule m_backLeft = new SwerveModule(Ports.BL_TURN, Ports.BL_DRIVE, Ports.BL_ENCODER);
    private final SwerveModule m_backRight = new SwerveModule(Ports.BR_TURN, Ports.BR_DRIVE, Ports.BR_ENCODER);

    // Establish Swerve Module States
    private SwerveModuleState[] moduleStates;

    // Setup NavX Gyro
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Establish Kinematics (the shape of the drivetrain)
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        Translation.FRONT_LEFT,
        Translation.FRONT_RIGHT,
        Translation.BACK_LEFT,
        Translation.BACK_RIGHT
    );

    // Establish Odometry (the position of each wheel)
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }
    );

    public Drivetrain() {
        setupDashboard();
    }

    private void setupDashboard(){
        SmartDashboard.putData("drivetrain/backright", m_backRight);
        SmartDashboard.putData("drivetrain/frontleft", m_frontLeft);
        SmartDashboard.putData("drivetrain/backleft", m_backLeft);
        SmartDashboard.putData("drivetrain/frontright", m_frontRight);
    }

    @Override
    public void periodic(){
        // Update Drivetrain Odometry
        m_odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );
    }

    public void resetGyro(){
        gyro.reset();
    }

    public void drive( double x, double y, double rot, boolean fieldRelative, double period){
                
        moduleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds( x, y, rot, gyro.getRotation2d() ) :
                new ChassisSpeeds(x,y,rot)
            , period)
        );
        
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.MAX_DRIVE_SPEED);

        m_frontLeft.setDesiredState(moduleStates[0]);
        m_frontRight.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);

    }

    @Override
    public void initSendable( SendableBuilder builder){
        builder.addDoubleProperty("current heading", gyro::getYaw, null);
    }

}
