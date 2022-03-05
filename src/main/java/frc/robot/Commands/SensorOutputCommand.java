package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeSensors;

public class SensorOutputCommand extends CommandBase {
    private IntakeSensors intakeSensors;

    public SensorOutputCommand(IntakeSensors intakeSensors) {
        this.intakeSensors = intakeSensors;
        addRequirements(intakeSensors);
    }

    public void execute() {
        intakeSensors.topTrigger();
        intakeSensors.bottomTrigger();
    }
}
