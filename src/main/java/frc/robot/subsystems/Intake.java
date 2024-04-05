package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final CANSparkMax motor;
    public final DigitalInput lowerSensor;
    public final DigitalInput upperSensor;

    public Intake(){
        motor = new CANSparkMax( Constants.Intake.Ports.MOTOR , MotorType.kBrushless);
        motor.setOpenLoopRampRate(Constants.Intake.RAMP_RATE);
        upperSensor = new DigitalInput(Constants.Intake.Ports.UPPER_SENSOR);
        lowerSensor = new DigitalInput(Constants.Intake.Ports.LOWER_SENSOR);
    }

    public Command runCommand(){
        return this.run(()->{
            motor.set(Constants.Intake.INTAKE_SPEED);
        })
            .finallyDo(()->stop())
            .withName("Run Intake");
    }

    public Command idleCommand(){
        return this.runOnce(()->{
            motor.set(0);
        }).withName("Idle Intake");
    }

    public boolean seesNote(){
        return !lowerSensor.get();
    }

    public boolean isLoaded(){
        return !upperSensor.get();
    }

    public void run() {
        motor.set(Constants.Intake.INTAKE_SPEED);
    }

    public void stop(){
        motor.set(0);
    }

    @Override
    public void initSendable( SendableBuilder builder ){
        super.initSendable(builder);
        builder.addBooleanProperty("Sees Note", this::seesNote, null);
        builder.addBooleanProperty("Loaded", this::isLoaded, null);
    }




}
