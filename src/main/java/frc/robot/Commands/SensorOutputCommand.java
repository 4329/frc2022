package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.IntakeSensors;

public class SensorOutput extends StartEndCommand {
    public SensorOutput(IntakeSensors subsystem) {
        super(subsystem::topTrigger, subsystem::bottomTrigger);
    }
}
