// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Autonomous;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static double autoTime;
  private SendableChooser<Autonomous> autoChooser = new SendableChooser<>();
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    autoChooser.addOption("Test", Autonomous.TEST);
    autoChooser.addOption("Do Nothing", Autonomous.DO_NOTHING);
    autoChooser.setDefaultOption("Do Nothing", Autonomous.DO_NOTHING);
    SmartDashboard.putData("Auto", autoChooser);

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   */
  @Override
  public void autonomousInit() {
    robotContainer.resetGyro();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // if ( autoChooser.getSelected() == Autonomous.DO_NOTHING ){

    // } else if ( autoChooser.getSelected() == Autonomous.SIMPLE_FORWARD ){
    //   robotContainer.autoMoveForward( Timer.getMatchTime() , getPeriod());
    // }
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //robotContainer.drive(getPeriod());
    //robotContainer.operate();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    //robotContainer.autoMoveForward(getPeriod());
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
  /* 
  private void driveWithJoystick(boolean fieldRelative){

  }
  */

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

}
