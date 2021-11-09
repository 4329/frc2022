package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystems.*;

public class ShooterDefault extends CommandBase {
    private final Shooter m_shooter;

    public ShooterDefault(Shooter shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooter.feedThroat();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopFeeder();
    }

}
