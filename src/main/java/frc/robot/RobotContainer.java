package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.*;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_robotDrive = new Drivetrain();
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -m_xspeedLimiter.calculate(quadraticTransform(applyDeadband(
                      m_driverController.getY(GenericHID.Hand.kLeft),DriveConstants.kInnerDeadband,
                      DriveConstants.kOuterDeadband)))*DriveConstants.kMaxSpeedMetersPerSecond,
                    -m_yspeedLimiter.calculate(quadraticTransform(applyDeadband(
                      m_driverController.getX(GenericHID.Hand.kLeft),DriveConstants.kInnerDeadband,
                      DriveConstants.kOuterDeadband)))*DriveConstants.kMaxSpeedMetersPerSecond,
                    -m_rotLimiter.calculate(quadraticTransform(applyDeadband(
                      m_driverController.getX(GenericHID.Hand.kRight),DriveConstants.kInnerDeadband,
                      DriveConstants.kOuterDeadband)))*DriveConstants.kMaxAngularSpeed,
                    true)
                    ,m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //Reset drivetrain when down on the DPad is pressed
    new POVButton(m_driverController, 180).whenPressed(() -> m_robotDrive.reset());
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0.0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(3.0, 1.0), new Translation2d(6.0, 0.0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(6.0, 5.0, new Rotation2d(Math.PI)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

            Trajectory exampleTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0.0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1.0, 2.0), new Translation2d(0.0, 4.0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3.0, 5.0, new Rotation2d(Math.PI)),
                config);


            SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                exampleTrajectory2,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
    
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return 
      swerveControllerCommand.andThen(
      () -> m_robotDrive.drive(0, 0, 0, false)).andThen(
      () -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0.0)))).andThen(
      swerveControllerCommand2.andThen(
      () -> m_robotDrive.drive(0, 0, 0, false))).andThen(
      () -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0.0))));
  }

  double applyDeadband(double input, double lowDeadband, double highDeadband)
  {
    if(Math.abs(input) < lowDeadband)
    {
      return 0.0;
    }
    else if(Math.abs(input) > highDeadband)
    {
      return Math.signum(input)*1.0;
    }
    else
    {
      return input;
    }
  }

  double quadraticTransform(double input){
    return Math.signum(input)*Math.pow(input, 2);
  }

  
}