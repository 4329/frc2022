package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Configrun;

public class IntakeSensors {
    //upper intake sensor
    DigitalInput shooterFeedTrigger = new DigitalInput(Configrun.get(1,"shooterFeedTriggerPort"));

    public boolean shooterFeedTrigger() {
        return shooterFeedTrigger.get();
    }
    //lower intake sensor
    DigitalInput storageIntakeTrigger = new DigitalInput(Configrun.get(3,"storageIntakeTriggerPort"));

    public boolean storageIntakeTrigger() {
        return storageIntakeTrigger.get();
    }
}