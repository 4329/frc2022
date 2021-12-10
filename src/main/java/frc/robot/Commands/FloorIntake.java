package frc.robot.Commands;

import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FloorIntake extends CommandBase {
    private final Intake m_intake;
    private final Timer m_timer = new Timer();
    private boolean m_ballIntaking = false;
    private double m_ballTime = 0.0;
    
    public FloorIntake(Intake intake){
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_intake.floorIntake();
        SmartDashboard.putNumber("Intake Current", m_intake.getCurrent());
        SmartDashboard.putNumber("Intake Velocity", m_intake.getMeasurement());
        if(m_timer.get() <= 0.25){
            m_intake.floorIntake();
        }
        else if(m_timer.get() > 0.25 && m_intake.getCurrent() >= 5.0 && !m_ballIntaking)
        {
            System.out.println("Ball Detected!");
            m_ballIntaking = true;
            m_ballTime = m_timer.get();
        }

        if(m_ballIntaking && m_timer.get()-m_ballTime > 0.10)
        {
            m_intake.stop();
        }
        else if(m_ballIntaking && m_timer.get()-m_ballTime > 0.25)
        {
            m_timer.reset();
            m_ballTime = 0.0;
            m_ballIntaking = false;
        }
    }
  
    @Override
    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
