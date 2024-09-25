package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeNote;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {

    // Initialize Controllers
    private final CommandJoystick driverJoystick = new CommandJoystick(0);
    private final CommandXboxController driverController = new CommandXboxController(Constants.Controllers.DRIVER);
    private final CommandXboxController operatorController = new CommandXboxController(Constants.Controllers.OPERATOR);

    // Initialize Subsystems
    public static Drivetrain swerve;
    public static Shooter shooter;
    public static Intake intake;
    public static Climber climber;

    // Auto Selector
    private final SendableChooser<Command> autonomousChooser;

    public RobotContainer() {

        swerve = new Drivetrain();
        shooter = new Shooter();
        intake = new Intake();
        climber = new Climber();

        NamedCommands.registerCommand("Aim Speaker", shooter.aimSpeakerCommand());
        NamedCommands.registerCommand("Manual Speaker", shooter.manualAimSpeakerCommand());
        NamedCommands.registerCommand("Score Speaker", shooter.scoreSpeakerCommand());
        NamedCommands.registerCommand("Intake Note", new IntakeNote());
        NamedCommands.registerCommand("Aim Intake", shooter.aimIntakeCommand());
        NamedCommands.registerCommand("Score Amp", shooter.aimSpeakerCommand());

        autonomousChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous", autonomousChooser);
        SmartDashboard.putData("drivetrain", swerve);
        SmartDashboard.putData("shooter", shooter);
        SmartDashboard.putData("intake", intake);
        SmartDashboard.putData("Score Speaker", shooter.scoreSpeakerCommand());
        SmartDashboard.putData("Aim Intake", shooter.aimIntakeCommand());

        setDefaultCommands();
        setDriverBindings();
        setOperatorBindings();

    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }

    private void setDefaultCommands() {

    }

    private void setOperatorBindings() {

        operatorController.a()
                .onTrue(shooter.aimAmpCommand())
                .onFalse(shooter.aimIntakeCommand())
                .and(operatorController.rightBumper())
                .onTrue(shooter.scoreAmpCommand())
                .onFalse(shooter.aimAmpCommand());

        operatorController.leftBumper()
                .onTrue(shooter.aimSpeakerCommand())
                .onFalse(shooter.aimIntakeCommand())
                .and(operatorController.rightBumper())
                .onTrue(shooter.scoreSpeakerCommand())
                .onFalse(shooter.aimIntakeCommand());

        operatorController.b()
                .onTrue(shooter.intakeNoteCommand().alongWith(intake.runCommand()))
                .onFalse(intake.idleCommand().alongWith(shooter.aimIntakeCommand()));

        operatorController.y()
                .onTrue(shooter.manualAimSpeakerCommand())
                .onFalse(shooter.aimIntakeCommand())
                .and(operatorController.rightBumper())
                .onTrue(shooter.scoreSpeakerCommand())
                .onFalse(shooter.aimIntakeCommand());

    }

    private void setDriverBindings() {

        boolean joystick = true;

        if (joystick) {
            swerve.setDefaultCommand( new RunCommand(() -> swerve.joystickDrive(
                driverJoystick.getX(),
                driverJoystick.getY(),
                driverJoystick.getZ()
            ), swerve));
        } else {
            swerve.setDefaultCommand(new RunCommand(() -> swerve.joystickDrive(
                    driverController.getLeftY(),
                    driverController.getLeftX(),
                    driverController.getRightX()), swerve));

            driverController.leftBumper()
                    .onTrue(shooter.intakeNoteCommand().alongWith(intake.runCommand()))
                    .onFalse(intake.idleCommand().alongWith(shooter.aimIntakeCommand()));

            driverController.rightBumper()
                    .onTrue(new InstantCommand(() -> {
                        swerve.kSlowMode = true;
                    }))
                    .onFalse(new InstantCommand(() -> {
                        swerve.kSlowMode = false;
                    }));

            driverController.back()
                    .onTrue(new InstantCommand(() -> swerve.resetGyro()));
        }

    }

    public void resetGyro() {
        swerve.resetGyro();
    }

}
