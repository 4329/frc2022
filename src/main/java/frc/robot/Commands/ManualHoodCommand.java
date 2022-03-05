package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.HoodSubsystem;

public class ManualHoodCommand extends CommandBase {
    private HoodSubsystem hoodSubsystem;

    public ManualHoodCommand(HoodSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
    }
    public void execute() {
        hoodSubsystem.HoodPeriodic();
    }

    @Override
    public boolean isFinished() {
        return hoodSubsystem.hoodSet();
    }
}
