// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.SwerveAlignment;
import frc.robot.Configrun;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private SwerveAlignment m_swerveAlignment;
  private Drivetrain drivetrain;

  private double coastWait;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    Configrun.loadconfig();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    drivetrain = new Drivetrain();
    m_robotContainer = new RobotContainer(drivetrain);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want run during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // SmartDashboard.putBoolean("Get Solenoid", m_Solenoid.get());
    m_robotContainer.robotPeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    drivetrain.brakeMode();
    coastWait = RobotController.getFPGATime();
    m_robotContainer.disableRobot();

  }

  @Override
  public void disabledPeriodic() {
    if (RobotController.getFPGATime() >= coastWait + 1000000) {
      drivetrain.coastMode();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.init();
    m_autonomousCommand = m_robotContainer.getAuto();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
     * switch(autoSelected) { case "My Auto": autonomousCommand = new
     * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
     * ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    m_robotContainer.autonomousPeriodic();
  }

  @Override
  public void autonomousExit() {
  Command resetForTeliOp = new InstantCommand (()->drivetrain.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(0.0))));
  resetForTeliOp.schedule();

  }

  @Override
  public void teleopInit() {
    m_robotContainer.init();
    drivetrain.brakeMode();
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
    m_robotContainer.teleopPeriodic();
    // m_Solenoid.set(SmartDashboard.getBoolean("Set Solenoid", false));
    // RobotContainer.limelightSubsystem = limeputDistance();

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    drivetrain.coastMode();

    if (m_swerveAlignment == null) {
      // This prevents 2 sets of widgets from appearing when disabling & enabling the
      // robot, causing a crash
      m_swerveAlignment = new SwerveAlignment(drivetrain);
      m_swerveAlignment.initSwerveAlignmentWidgets();
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_swerveAlignment.updateSwerveAlignment();
    m_robotContainer.test();

  }
}