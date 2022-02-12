package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class ShooterCommand extends CommandBase {
    
    ShooterFeedSubsytem shooterFeedSubsystem;
    StorageIntake storageIntake;
    ShooterFeedSubsytem shooterFeed;
    IntakeSensors intakeSensors;

    public ShooterCommand(ShooterFeedSubsytem shooterFeed, IntakeSensors intakeSensors) {

        this.shooterFeed = shooterFeedSubsystem;
        this.intakeSensors = intakeSensors;
    }

    @Override
    public void initialize() {

        storageIntake.storageIntakeIn();
        shooterFeed.shooterFeedUp();
    }

    @Override
    public void execute() {
        if (!intakeSensors.topTrigger()) {
            shooterFeed.shooterFeedStop();

            if (!intakeSensors.bottomTrigger()) {
                storageIntake.storageIntakeStop();
                shooterFeed.shooterFeedStop();
            }
        }

    }

    @Override
    public boolean isFinished() {

        return false;
    }

}
