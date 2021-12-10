package frc.robot.Commands;

import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Intake m_intake;
    private final Timer m_intakeTimer = new Timer();
    
    public FeedShooter(Shooter shooter, Turret turret, Intake intake){
        m_shooter = shooter;
        m_turret = turret;
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intakeTimer.reset();
        m_intakeTimer.start();
    }

    @Override
    public void execute() {
        if(m_turret.visionAligned() && m_shooter.atSetpoint()){
            m_shooter.runFeeder();
        }
        else{
            m_shooter.stopFeeder();
        }
        if(m_intakeTimer.get() > 0.50){
            m_intakeTimer.reset();
        }
        else if(m_intakeTimer.get() > 0.25){
            m_intake.feedIn();
        }
        
    }
  
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopFeeder();
        m_intake.stop();
        m_intakeTimer.stop();
    }
}
