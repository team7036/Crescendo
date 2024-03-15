package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveDistance extends Command{
    private Drivetrain m_drivetrain;
    private Vision m_vision;
    private double m_amount;
    private ProfiledPIDController m_controller = new ProfiledPIDController(0.1, 0.02, 0, new Constraints(3, 2));

    public DriveDistance (Drivetrain drivetrain, Vision vision, double amount, double tolerance) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_amount = amount;
        m_controller.setTolerance(tolerance);
        m_controller.setIZone(0.4);
        addRequirements(drivetrain);
    }

    // @Override
    // public void initialize() {
        
    // }

    @Override
    public void execute() {
        //m_drivetrain.drive(0, 1, m_amount, m_amount);
    }
}
