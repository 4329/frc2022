package frc.robot.Commands;

import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Swerve.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoalShoot extends CommandBase {
  private final Shooter m_shooter;
  private final Turret m_turret;
  private final Drivetrain m_robotDrive;

  public GoalShoot(Shooter shooter, Turret turret, Drivetrain drive) {
    m_shooter = shooter;
    m_turret = turret;
    m_robotDrive = drive;
  }

  @Override
  public void initialize() {
    m_shooter.enable();
    m_turret.enable();
    m_turret.trackTarget(true);
  }

  @Override
  public void execute() {
    m_shooter.setRPM(Limelight.getDistance());
    m_turret.setAngle(m_robotDrive.getGyro().getRadians());
    SmartDashboard.putNumber("Shooter Current", m_shooter.getTotalCurrent());
    SmartDashboard.putNumber("Shooter RPMs", m_shooter.getMeasurement());
    SmartDashboard.putNumber("Shooter Error", m_shooter.getMeasurement()-m_shooter.getSetpoint());
    SmartDashboard.putNumber("Distance", Limelight.getDistance());
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.disable();
    m_turret.trackTarget(false);
  }

}
