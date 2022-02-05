package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
public class IntakeSensorsLogic extends CommandBase {
    private IntakeSensors intakeSensors;
    private ShooterFeedSubsytem shooterFeed;
    private StorageIntake storageIntake;

    public IntakeSensorsLogic(IntakeSensors intakeSensors, ShooterFeedSubsytem shooterFeed, StorageIntake storageIntake) {
        this.intakeSensors = intakeSensors;
        this.shooterFeed = shooterFeed;
        this.storageIntake = storageIntake;
    }

    public void sensorLogic() {
        if(intakeSensors.topTrigger()) {
            System.out.println("top deactivated");

            if(intakeSensors.bottomTrigger()) {
                System.out.println("bottom deactivated");
            }
        }
    }
}
