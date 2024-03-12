package frc.robot.subsystems.drivetrain;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {

    // Create Swerve Module Objects
    private final SwerveModule m_frontLeft = new SwerveModule(Ports.FL_TURN, Ports.FL_DRIVE, Ports.FL_ENCODER);
    private final SwerveModule m_frontRight = new SwerveModule(Ports.FR_TURN, Ports.FR_DRIVE, Ports.FR_ENCODER);
    private final SwerveModule m_backLeft = new SwerveModule(Ports.BL_TURN, Ports.BL_DRIVE, Ports.BL_ENCODER);
    private final SwerveModule m_backRight = new SwerveModule(Ports.BR_TURN, Ports.BR_DRIVE, Ports.BR_ENCODER);

    // Establish Swerve Module States
    private SwerveModuleState[] moduleStates;

    // Setup NavX Gyro
    /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

    // private static AHRS gyro = new AHRS(SPI.Port.kMXP);

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
        ahrs.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }
    );

    public Drivetrain() {
        ahrs.zeroYaw();
        setupDashboard();
    }

    private void setupDashboard(){
        SmartDashboard.putData("drivetrain/backright", m_backRight);
        SmartDashboard.putData("drivetrain/frontleft", m_frontLeft);
        SmartDashboard.putData("drivetrain/backleft", m_backLeft);
        SmartDashboard.putData("drivetrain/frontright", m_frontRight);
    }

    public Command driveToPosition( double x, double y, double pose ){
        return run(()->{

        });
    }

    @Override
    public void periodic(){
        // Update Drivetrain Odometry
        m_odometry.update(
            ahrs.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );
    }

    public void drive( double x, double y, double rot){
        
        // ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
        
        moduleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize( new ChassisSpeeds(x, y, rot), 0.02 )
        );
        
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, frc.robot.Constants.Drivetrain.MAX_DRIVE_SPEED);

        m_frontLeft.setDesiredState(moduleStates[0]);
        m_frontRight.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);

    }

    public void rotateToSpeaker() {
        //double currentYaw = Vision.botpose[5];
    }

    @Override
    public void initSendable( SendableBuilder builder){
    }

}
