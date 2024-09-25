package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain.*;


public class Drivetrain extends SubsystemBase {

    public boolean kSlowMode = false;

    // Create Swerve Module Objects
    private final SwerveModule m_frontLeft = new SwerveModule(Ports.FL_TURN, Ports.FL_DRIVE, Ports.FL_ENCODER);
    private final SwerveModule m_frontRight = new SwerveModule(Ports.FR_TURN, Ports.FR_DRIVE, Ports.FR_ENCODER);
    private final SwerveModule m_backLeft = new SwerveModule(Ports.BL_TURN, Ports.BL_DRIVE, Ports.BL_ENCODER);
    private final SwerveModule m_backRight = new SwerveModule(Ports.BR_TURN, Ports.BR_DRIVE, Ports.BR_ENCODER);

    // Create Rate Limiters
    private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotSpeedLimiter = new SlewRateLimiter(2 * Math.PI);

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
        gyro.getRotation2d().unaryMinus(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }
    );

    public Drivetrain() {

        gyro.reset();

        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    Constants.Drivetrain.MAX_DRIVE_SPEED, // Max module speed, in m/s
                    0.762, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
        );

    }

    public void joystickDrive( double x, double y, double rot ){

        double xSpeed = xSpeedLimiter.calculate( MathUtil.applyDeadband( x, 0.05 ) ) * ( kSlowMode ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED);
        double ySpeed = -ySpeedLimiter.calculate( MathUtil.applyDeadband( y, 0.05 ) ) * ( kSlowMode ? Constants.Drivetrain.SLOW_DRIVE_SPEED : Constants.Drivetrain.MAX_DRIVE_SPEED );
        double rotSpeed = -rotSpeedLimiter.calculate( MathUtil.applyDeadband( rot, 0.05 ) ) *  (kSlowMode ? Constants.Drivetrain.SLOW_TURN_SPEED : Constants.Drivetrain.MAX_TURN_SPEED);
            
        this.drive( new ChassisSpeeds(xSpeed, ySpeed, rotSpeed) );

    }

    public void resetPose(Pose2d pose){
        m_odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds( getModuleStates() );
    }

    private SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
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
        return m_odometry.getPoseMeters();
    }

    public ChassisSpeeds getCurrentSpeeds(){
        return m_kinematics.toChassisSpeeds(moduleStates);
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

    public void driveRobotRelative( ChassisSpeeds speeds){
        moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.MAX_DRIVE_SPEED);
        m_frontLeft.setDesiredState(moduleStates[0]);
        m_frontRight.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);
    }

    public void drive( ChassisSpeeds speeds ){

        moduleStates = m_kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d().unaryMinus()));
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
        SmartDashboard.putData("drivetrain/frontleft", m_frontLeft);
        SmartDashboard.putData("drivetrain/frontright", m_frontRight);

        builder.addDoubleProperty("Yaw", gyro::getYaw, null);
        builder.addDoubleProperty("x", this.getPose()::getX, null);
        builder.addDoubleProperty("y", this.getPose()::getY, null);
    }

}
