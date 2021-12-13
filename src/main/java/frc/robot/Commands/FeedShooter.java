package frc.robot.Commands;

import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeedShooter extends CommandBase {
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Intake m_intake;
    private final Timer m_intakeTimer = new Timer();
    private final Timer m_unjamTimer = new Timer();
    private double shotBallTime = 0.0;
    private double adjShotBallTime = 0.0;
    private boolean shotBall = false;
    private boolean firstBallShot = false;
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
        SmartDashboard.putBoolean("First Ball Shot", firstBallShot);
        if(shotBall && m_unjamTimer.get() - shotBallTime > 0.15){
            shotBall = false;
        }
        else if(!shotBall && m_shooter.getTotalCurrent() > 35.0 && ready){
            shotBall = true;
            shotBallTime = m_unjamTimer.get();
            if(firstBallShot){
                adjShotBallTime = shotBallTime;
            }
            else{
                adjShotBallTime = shotBallTime-0.50;
                firstBallShot = true;
            }
        }
        
        if(ready && m_unjamTimer.get() - adjShotBallTime < 0.75){
            m_shooter.runFeeder();
        }
        else if(m_unjamTimer.get() - adjShotBallTime >= 0.75+0.25)
        {
            m_unjamTimer.reset();
            shotBallTime = 0.0;
            adjShotBallTime = 0.0;
        }
        else if(m_unjamTimer.get() - adjShotBallTime >= 0.75)
        {
            m_shooter.reverseFeeder();
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
        shotBallTime = 0.0;
        adjShotBallTime = 0.0;
        shotBall = false;
        firstBallShot = false;
    }
}
 