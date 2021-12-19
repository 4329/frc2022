package frc.robot.Utilities;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.lang.String;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Swerve.Drivetrain;


public class AutoFromTrajectory {
  static Trajectory m_trajectory = new Trajectory();
  static Drivetrain m_drive;
  public static Command autoCSVCommand(String CSV, Drivetrain drive) {
    m_drive = drive;
    generate(CSV);
    
    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(m_trajectory,
        m_drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController, m_drive::setModuleStates, m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(m_trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand;
  }

  public static Command autoTrajectoryCommand(Trajectory trajectory, Drivetrain drive) {
    m_drive = drive;
    
    var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
        m_drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0), new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController, m_drive::setModuleStates, m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand;
  }

  private static void generate(String CSV){
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(CSV);
        List<String> lines = Files.readAllLines(trajectoryPath);
        int j = 0;
        double[] elements = new double[lines.size()*7];
        for(String line : lines){
          String[] parts = line.split(",");
          for (String part : parts){
            elements[j++] = Double.parseDouble(part);
          }
        }
    
        // Create a list of states from the elements.
        List<Trajectory.State> states = new ArrayList<>();
        for (int i = 0; i < elements.length; i += 7) {
          states.add(
              new Trajectory.State(
                  elements[i],
                  elements[i + 1],
                  elements[i + 2],
                  new Pose2d(elements[i + 3], 8.2296 - elements[i + 4], new Rotation2d(-1.0*elements[i + 5])),
                  elements[i + 6]));
        }
  
        m_trajectory = new Trajectory(states);
     } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + CSV, ex.getStackTrace());
     }
    }
}
