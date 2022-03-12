package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;

public class ManualHoodCommand extends CommandBase {
    private HoodSubsystem hoodSubsystem;
    private Shooter shooter;

    public ManualHoodCommand(HoodSubsystem hoodSubsystem, Shooter shooter) {
        this.hoodSubsystem = hoodSubsystem;
        this.shooter = shooter;
    }
    
    public void execute() {
        hoodSubsystem.HoodPeriodic(shooter);
    }

    @Override
    public boolean isFinished() {
        return hoodSubsystem.hoodSet();
    }
    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.stop();
    }
}
