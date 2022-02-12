package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.StorageIntake;

public class StorageIntakeOutCommand extends StartEndCommand {
    public StorageIntakeOutCommand(StorageIntake subsystem) {
        
        super(subsystem::storageIntakeOut, subsystem::storageIntakeStop);
    }
}