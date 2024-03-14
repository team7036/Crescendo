package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Shooter.FlyWheels.Mode;
import frc.robot.subsystems.shooter.FlyWheels;

public class FlywheelSpeed extends Command {

    private final FlyWheels m_flywheels;
    private final DoubleSupplier m_rpm;

    public FlywheelSpeed(FlyWheels subsystem, DoubleSupplier rpm){
        m_flywheels = subsystem;
        m_rpm = rpm;
        addRequirements(m_flywheels);
    }

    @Override
    public void initialize(){
        m_flywheels.setSpeed(m_rpm.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return m_flywheels.atSpeed(m_rpm.getAsDouble());
    }
}
