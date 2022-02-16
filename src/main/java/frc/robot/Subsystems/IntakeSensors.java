package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class IntakeSensors extends SubsystemBase {
    private NetworkTableEntry topTriggerStatus;
    private NetworkTableEntry bottomTriggerStatus;

    DigitalInput topTrigger = new DigitalInput(Configrun.get(1, "topTriggerPort"));
    DigitalInput bottomTrigger = new DigitalInput(Configrun.get(3, "bottomTriggerPort"));

    public IntakeSensors() {
        topTriggerStatus = Shuffleboard.getTab("Competition Info").add("Top Trigger", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        bottomTriggerStatus = Shuffleboard.getTab("Competition Info").add("Bottom Trigger", true).withPosition(0, 1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
    }

    public boolean topTrigger() {
        topTriggerStatus.setBoolean(topTrigger.get());

        return topTrigger.get();
    }

    public boolean bottomTrigger() {
        bottomTriggerStatus.setBoolean(bottomTrigger.get());

        return bottomTrigger.get();
    }
}
