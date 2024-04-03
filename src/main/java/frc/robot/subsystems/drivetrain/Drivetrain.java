package frc.robot.subsystems.drivetrain;
import java.util.concurrent.TimeUnit;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
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

      private SwerveDrivePoseEstimator m_poseEstimator;


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

        // AutoBuilder.configureHolonomic(
        //     this::getPose, // Robot pose supplier
        //     this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        //     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        //             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        //             new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
        //             15.1, // Max module speed, in m/s
        //             0.762, // Drive base radius in meters. Distance from robot center to furthest module.
        //             new ReplanningConfig() // Default path replanning config. See the API for the options here
        //     ),
        //     () -> {
        //       // Boolean supplier that controls when the path will be mirrored for the red alliance
        //       // This will flip the path being followed to the red side of the field.
        //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //       var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;
        //     },
        //     this // Reference to this subsystem to set requirements
        // );
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return new ChassisSpeeds();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    private void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
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

    public void drive(double x, double y, double rot, boolean fieldRelative, double period){
                
        moduleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds( -x, -y, rot, gyro.getRotation2d().unaryMinus() ) :
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
        SmartDashboard.putData("drivetrain/backright", m_backRight);
        SmartDashboard.putData("drivetrain/backleft", m_backLeft);
        SmartDashboard.putData("drivetrain/frontright", m_frontRight);
        SmartDashboard.putData("drivetrain/frontleft", m_frontLeft);
        builder.addDoubleProperty("current heading", gyro::getYaw, null);
    }

}
