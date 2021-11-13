package frc.robot.Commands;

import frc.robot.Constants.*;
import frc.robot.Subsystems.Swerve.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveByController extends CommandBase {
  private final Drivetrain m_robotDrive;
  private final XboxController m_controller;
  private boolean fieldOrient = true;

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
