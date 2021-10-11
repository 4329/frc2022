package frc.robot.Commands;

import frc.robot.Utilities.*;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Swerve.*;
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
    m_shooter.setRPM(m_turret.getDistance());
    m_turret.setAngle( MathUtils.toUnitCircAngle(3 * Math.PI / 2.0 +m_robotDrive.getGyro().getRadians()));
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.disable();
    m_turret.trackTarget(false);
  }

}
