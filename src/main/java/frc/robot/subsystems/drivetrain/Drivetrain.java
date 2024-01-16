package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;

public class Drivetrain {

    // Establish Drivetrain Constants
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;

    // Establish Drivetrain Geometry
    private final Translation2d m_frontLeftLocation = new Translation2d(0, 0);
    private final Translation2d m_frontRightLocation = new Translation2d(0, 0);
    private final Translation2d m_backLeftLocation = new Translation2d(0, 0);
    private final Translation2d m_backRightLocation = new Translation2d(0, 0);

    // Create Swerve Module Objects
    private final SwerveModule m_frontLeft = new SwerveModule(23, 22, 0);
    private final SwerveModule m_frontRight = new SwerveModule(17, 16, 0);
    private final SwerveModule m_backLeft = new SwerveModule(21, 20, 0);
    private final SwerveModule m_backRight = new SwerveModule(19, 18, 0);

    public Drivetrain() {
    
    }
}
