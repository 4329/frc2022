package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.StorageIntake;

public class StorageIntakeInCommand extends StartEndCommand {
    public StorageIntakeInCommand(StorageIntake subsystem) {
        super(subsystem::storageIntakeIn, subsystem::storageIntakeStop);
    }
}