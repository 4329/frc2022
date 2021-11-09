package frc.robot.Commands;

import frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    
    public FeedShooter(Shooter shooter, Turret turret){
        m_shooter = shooter;
        m_turret = turret;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(m_turret.visionAligned() && m_shooter.atSetpoint()){
            m_shooter.runFeeder();
        }
        else{
            m_shooter.stopFeeder();
        }
    }
  
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopFeeder();
    }
}
