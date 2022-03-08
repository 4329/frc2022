package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class IntakeSensors extends SubsystemBase {

    private final NetworkTableEntry topTriggerStatus;
    private final NetworkTableEntry bottomTriggerStatus;

    private final DigitalInput topTrigger;
    private final DigitalInput bottomTrigger;

    public IntakeSensors() {

        topTrigger = new DigitalInput(Configrun.get(1, "TopSensorPort"));
        bottomTrigger = new DigitalInput(Configrun.get(3, "BottomSensorPort"));

        topTriggerStatus = Shuffleboard.getTab("RobotData2").add("Top Sensor", true).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
        bottomTriggerStatus = Shuffleboard.getTab("RobotData2").add("Bottom Sensor", true).withPosition(0, 1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
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
