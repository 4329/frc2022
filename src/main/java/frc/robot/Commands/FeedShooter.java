package frc.robot.Commands;

import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Intake m_intake;
    private final Timer m_intakeTimer = new Timer();
    private final Timer m_unjamTimer = new Timer();
    private double feedBallTime = 0.0;
    private boolean fedFirstBall = false;
    private boolean ready = false;
    
    public FeedShooter(Shooter shooter, Turret turret, Intake intake){
        m_shooter = shooter;
        m_turret = turret;
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intakeTimer.reset();
        m_intakeTimer.start();
        m_unjamTimer.reset();
        m_unjamTimer.start();
    }

    @Override
    public void execute() {
        ready = m_turret.visionAligned() && m_shooter.atSetpoint();
        if(ready && !fedFirstBall){
            m_shooter.runFeeder();
            feedBallTime = m_unjamTimer.get();
            fedFirstBall = true;
        }
        else if(m_unjamTimer.get() - feedBallTime > 0.30 && m_unjamTimer.get()-feedBallTime < 0.50)
        {
            m_shooter.reverseFeeder();
        }
        else if(ready)
        {
            m_shooter.runFeeder();
        }
        else{
            m_shooter.stopFeeder();
        }
        if(m_intakeTimer.get() > 0.50){
            m_intakeTimer.reset();
            m_intake.disable();
        }
        else if(m_intakeTimer.get() > 0.25){
            m_intake.enable();
            m_intake.feedIn();
        }
        
    }
  
    @Override
    public void end(boolean interrupted) {
        m_shooter.stopFeeder();
        m_intake.disable();
        fedFirstBall = false;
    }
}
