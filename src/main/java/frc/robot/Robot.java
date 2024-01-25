// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static Drivetrain m_swerve = new Drivetrain();
  //private static Shooter m_shooter = new Shooter(0, 0);
  //private static Intake m_intake = new Intake(0, 0);
  private static XboxController m_controller = new XboxController(0);

  private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_ySpeedlimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    driveWithJoystick(true);

  }

  private void driveWithJoystick(boolean fieldRelative){
    // getting x speed. inverted bc xbox controllers return negative
    // values when pushed forward
    final var xSpeed =
      -m_xSpeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.02)) * Drivetrain.kMaxSpeed;

    // getting y speed or sideways/strafe speed. inverting because we want a
    // positive value when we pull to the left
    // xbox controllers return positive value when pulled to the right
    final var ySpeed =
      -m_ySpeedlimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.02)) * Drivetrain.kMaxSpeed;

    // getting rate of angular rotation. inverting bc we want a positive value
    // when pulled to the left (CCW is positive in math). xbox controllers return
    // positive value when pulled to the right 
    final var rot =
      -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02)) * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}