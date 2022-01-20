// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //Declare the joystick axes and buttons
  final Joystick controller = new Joystick(0);
  int lJoyX = 0;
  int lJoyY = 1;
  int lTrigger = 7;
  int rTrigger = 8;
  int rJoyX = 2;
  int rJoyY = 3 ;
  int a = 2;
  int b = 3;
  int x = 1;
  int y = 4;
  int lb = 5;
  int rb = 6;
  int back = 9;
  int start = 10;
  int lJoy = 11;
  int rJoy = 12;

  //Declare the drive train motors
  private final WPI_VictorSPX m_leftMasterMotor = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_leftSlaveMotor = new WPI_VictorSPX(2);
  private final WPI_VictorSPX m_rightMasterMotor = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_rightSlaveMotor = new WPI_VictorSPX(4);

  // Index and shooter motors
  private final WPI_VictorSPX m_indexMotor = new WPI_VictorSPX(5);
  private final WPI_VictorSPX m_shooterMotor = new WPI_VictorSPX(6);

  //Declare the motor groups
  private SpeedControllerGroup m_left = new SpeedControllerGroup(m_leftMasterMotor, m_leftSlaveMotor);
  private SpeedControllerGroup m_right = new SpeedControllerGroup(m_rightMasterMotor, m_rightSlaveMotor);

  //Start the differential drive
  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_left, m_right);

  //Declare the X and Y offsets that can be changed with left and right D-Pad
  int xOffSet = 1;
  int yOffSet = 1;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_right.setInverted(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  private double startTime;

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
    double elapsed = time - startTime;
    System.out.print("Auton online for " + (int) elapsed + "s.");
    if (elapsed < 5) {
      m_robotDrive.arcadeDrive(0, 0);
      m_shooterMotor.set(1);
      m_indexMotor.set(0);
      System.out.println(" Winding up");
    } else if (elapsed < 8 && elapsed > 5) {
      m_robotDrive.arcadeDrive(0, 0);
      m_shooterMotor.set(1);
      m_indexMotor.set(1);
      System.out.println(" Firing.");
    } else if (elapsed < 11 && elapsed > 8) {
      m_robotDrive.arcadeDrive(-0.2, 0);
      m_shooterMotor.set(0);
      m_indexMotor.set(0);
      System.out.println(" Driving.");
    } else if (elapsed > 11) {
      m_robotDrive.arcadeDrive(0, 0);
      m_shooterMotor.set(0);
      m_indexMotor.set(0);
      System.out.println(" Auton complete.");
    }

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(controller.getRawAxis(1) * yOffSet, controller.getRawAxis(2) * xOffSet);  //Initialize the drive with the joysticks
    m_indexMotor.set(rTrigger);
    m_shooterMotor.set(controller.getRawAxis(3));
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    System.out.println(controller.getRawAxis(0));

  }
}