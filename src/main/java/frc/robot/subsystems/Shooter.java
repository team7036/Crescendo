package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.Mode;

public class Shooter extends SubsystemBase {

    public Mode mode = Mode.IDLE;

    // Motors
    private final CANSparkMax motor;
    private final CANSparkMax bottomMotor;
    // Encoder
    private final RelativeEncoder encoder;
    // Shooter Arm
    private final Arm arm;
    // Staging Servo
    private final Servo stagingServo;
    // Inputs
    private DigitalInput sensor;

    public Shooter(){
        // Motors
        motor = new CANSparkMax(Constants.Shooter.Ports.TOP_MOTOR, MotorType.kBrushless);
        motor.setInverted(true);
        bottomMotor = new CANSparkMax(Constants.Shooter.Ports.BOTTOM_MOTOR, MotorType.kBrushless);
        bottomMotor.follow(motor);
        // Encoder
        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(Constants.Shooter.SPEED_CONVERSION);
        // Arm
        arm = new Arm();
        // Setup Staging Servo and Sensor
        stagingServo = new Servo(Constants.Shooter.Ports.STAGING_SERVO);
        sensor = new DigitalInput(Constants.Shooter.Ports.SENSOR);
    }

    public Command setAngleCommand( double angle ){
        return this.run(()->{});
    }

    public void setAngle( double angle ){
        /*
         * 3 ft -> 1.894
         */
        arm.setAngle(angle);
    }

    @Override
    public void periodic(){
        if ( mode == Mode.INTAKE ) {
            setAngle(0);
            motor.set(-0.25);
            stagingServo.setAngle(180);
        } else if ( mode == Mode.MANUAL_AIM ) {
            setAngle(1.5);
            motor.set(0);
            stagingServo.setAngle(90);
        } else if ( mode == Mode.FIRE ) {
            setAngle(1.5);
            motor.set(1);
            if ( encoder.getVelocity() < 2500 ){
                stagingServo.setAngle(90);
            } else {
                stagingServo.setAngle(0);
            }
        } else {
            setAngle(0);
            motor.set(0);
            stagingServo.setAngle(90);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("angle", arm::getMeasurement, null);
    }



}
