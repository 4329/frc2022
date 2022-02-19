package frc.robot.Commands;

import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Swerve.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoalShoot extends CommandBase {
  private final Turret m_turret;

  public GoalShoot(Turret turret) {
    m_turret = turret;
  }

  @Override
  public void initialize() {
    m_turret.enable();
    //m_turret.setAngle(Math.PI / 2);
    m_turret.trackTarget(true);
    System.out.println("track target yes");
  }

  @Override
  public void execute() {
    //SmartDashboard.putNumber("Distance", Limelight.getDistance());
  }

  @Override
  public void end(boolean interrupted) {
    m_turret.trackTarget(false);
    System.out.println("track target no");
  }

}