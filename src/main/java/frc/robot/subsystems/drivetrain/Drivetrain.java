package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.struct.Translation2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
//import frc.robot.subsystems.drivetrain.SwerveModule;

public class Drivetrain {
    private final Translation2d m_frontLeftLocation = new Translation2d(0, 0);
    private final Translation2d m_frontRightLocation = new Translation2d();
    private final Translation2d m_backLeftLocation = new Translation2d();
    private final Translation2d m_backRightLocation = new Translation2d();

    private final SwerveModule m_frontLeft = new SwerveModule();
    private final SwerveModule m_frontRight = new SwerveModule();
    private final SwerveModule m_backLeft = new SwerveModule();
    private final SwerveModule m_backRight = new SwerveModule();
    public Drivetrain() {
    
    }
}
