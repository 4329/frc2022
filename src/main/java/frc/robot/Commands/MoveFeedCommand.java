package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class MoveFeedCommand extends CommandBase {
    
    ShooterFeedSubsytem shooterFeedSubsytem;
    StorageIntake storageIntake;

    public MoveFeedCommand(ShooterFeedSubsytem shooterFeedSubsytem, StorageIntake storageIntake) {

        this.storageIntake = storageIntake;
        this.shooterFeedSubsytem = shooterFeedSubsytem;
        addRequirements(storageIntake, shooterFeedSubsytem);
    }

    @Override
    public void execute() {

        storageIntake.storageIntakeInSlow();
        shooterFeedSubsytem.shooterFeedFire();
    }

    @Override
    public void end(boolean interrupted) {

        storageIntake.storageIntakeStop();
        shooterFeedSubsytem.shooterFeedStop();
    }
}
