package frc.robot.Commands;

import com.fasterxml.jackson.annotation.OptBoolean;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;


public class TowerLowCommand extends CommandBase {
    StorageIntake storageIntake;
    ShooterFeedSubsytem shooterFeed;
    Shooter shooter;
    HoodSubsystem hood;

    public TowerLowCommand(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hood) {
        this.storageIntake = storageIntake;
        this.shooterFeed = shooterFeed;
        this.shooter = shooter;
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setPosition(HoodPosition.HALF);
        shooter.shoot(1400);

        if (shooter.getShooterError()) {

            storageIntake.storageIntakeInSlow();
            shooterFeed.shooterFeedFire();
        } else {

            storageIntake.storageIntakeStop();
            shooterFeed.shooterFeedStop();
        }
    }

    @Override
    public void end(boolean interrupt) {
        storageIntake.storageIntakeStop();
        shooterFeed.shooterFeedStop();
        shooter.holdFire();
        hood.setPosition(HoodPosition.OPEN);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
