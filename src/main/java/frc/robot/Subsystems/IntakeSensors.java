package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Configrun;

public class IntakeSensors {
    //upper intake sensor
    DigitalInput topTrigger = new DigitalInput(Configrun.get(1,"shooterFeedTriggerPort"));

    public boolean topTrigger() {
        return topTrigger.get();
    }
    //lower intake sensor
    DigitalInput bottomTrigger = new DigitalInput(Configrun.get(3,"storageIntakeTriggerPort"));

    public boolean bottomTrigger() {
        return bottomTrigger.get();
    }
}