package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.Mode;
import frc.robot.subsystems.Vision;

public class Shooter extends SubsystemBase {

    public Mode mode = Mode.IDLE;
    // Shooter Flywheels
    public final FlyWheels flyWheels;
    // Shooter Arm
    public final Arm arm;
    // Staging Servo
    public final Servo stagingServo;

    public Shooter(){
        // flywheels
        flyWheels = new FlyWheels();
        // Arm
        arm = new Arm();
        // Setup Staging Servo and Sensor
        stagingServo = new Servo(Constants.Shooter.Ports.STAGING_SERVO);
    }

    public void intake(){
        flyWheels.setSpeed(Constants.Shooter.FlyWheels.INTAKE_SPEED);
        stagingServo.setAngle(Constants.Shooter.StagingServo.INTAKE_SPEED);
    }

    public void stop(){
        flyWheels.setSpeed(0);
        stagingServo.setAngle(Constants.Shooter.StagingServo.STOP_SPEED);
    }

    public void amp() {
        flyWheels.setSpeed(Constants.Shooter.FlyWheels.AMP_SPEED);
        stagingServo.setAngle(Constants.Shooter.StagingServo.FIRE_SPEED);
    }

    @Override
    public void initSendable(SendableBuilder builder){
        SmartDashboard.putData("shooter/pid", arm.getController());
        SmartDashboard.putData("arm",arm);
        SmartDashboard.putData("flywheels", flyWheels);
        builder.addDoubleProperty("calculatedAngle", Vision::calculateArmAngle, null);      
    }



}
