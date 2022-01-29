package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.Swerve.StorageIntake;

public class StorageIntakeCommand extends StartEndCommand {
    public StorageIntakeCommand(StorageIntake subsystem) {
        super(subsystem::storageIntakeIn, subsystem::storageIntakeStop, subsystem, subsystem);
    }
}