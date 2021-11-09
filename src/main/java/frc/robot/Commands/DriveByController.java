package frc.robot.Commands;

import frc.robot.Constants.*;
import frc.robot.Subsystems.Swerve.*;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveByController extends CommandBase {
  private final Drivetrain m_robotDrive;
  private final XboxController m_controller;
  private boolean fieldOrient = true;
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kSlewRate);

  public DriveByController(Drivetrain drive, XboxController controller) {
    m_robotDrive = drive;
    m_controller = controller;
    addRequirements(m_robotDrive);
  }

  @Override
  public void execute() {
    m_robotDrive.drive(
        -quadraticTransform(applyDeadband(m_controller.getY(GenericHID.Hand.kLeft)))
            * DriveConstants.kMaxSpeedMetersPerSecond,
        -quadraticTransform(applyDeadband(m_controller.getX(GenericHID.Hand.kLeft)))
            * DriveConstants.kMaxSpeedMetersPerSecond,
        -quadraticTransform(applyDeadband(m_controller.getX(GenericHID.Hand.kRight)))
            * DriveConstants.kMaxAngularSpeed,
        fieldOrient);
        SmartDashboard.putNumber("Robot Velocity X", m_robotDrive.getChassisSpeed().vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Velocity Y", m_robotDrive.getChassisSpeed().vyMetersPerSecond);
        SmartDashboard.putNumber("Robot Velocity Z", m_robotDrive.getChassisSpeed().omegaRadiansPerSecond);

  }

  public void changeFieldOrient() {
    if (fieldOrient) {
      fieldOrient = false;
    } else {
      fieldOrient = true;
    }

  }

  private double applyDeadband(double input) {
    if (Math.abs(input) < DriveConstants.kInnerDeadband) {
      return 0.0;
    } else if (Math.abs(input) > DriveConstants.kOuterDeadband) {
      return Math.signum(input) * 1.0;
    } else {
      return input;
    }
  }

  private double quadraticTransform(double input) {
    return Math.signum(input) * Math.pow(input, 2);
  }

}
