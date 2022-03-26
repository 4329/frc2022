package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;

public class HoodToOpenCommand extends CommandBase {
    private HoodSubsystem hoodSubsystem;

    public HoodToOpenCommand(HoodSubsystem hoodSubsystem, Shooter shooter) {
        
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(hoodSubsystem);
    }
    
    public void initialize() {

        hoodSubsystem.setPosition(HoodPosition.OPEN);
    }

}
