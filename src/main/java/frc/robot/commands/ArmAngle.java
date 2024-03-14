package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Arm;

public class ArmAngle extends Command {

    private final Arm m_arm;
    private final double m_angle;

    public ArmAngle(Arm subsystem, double angle){
        m_arm = subsystem;
        m_angle = angle;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.getController().setGoal(m_angle);
    }

    @Override
    public boolean isFinished(){
        return m_arm.getController().atGoal();
    }
}
