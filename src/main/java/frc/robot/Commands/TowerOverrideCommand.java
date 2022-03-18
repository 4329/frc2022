package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;

public class TowerOverrideCommand extends CommandBase {

    final StorageIntake storageIntake;
    final ShooterFeedSubsytem shooterFeed;
    final Shooter shooter;
    final HoodSubsystem hood;

    double setpoint;

    /**
     * Runs the tower intake and shooter
     * 
     * @param storageIntake
     * @param shooterFeed
     * @param shooter
     * @param hood
     */
    public TowerOverrideCommand(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hood) {

        this.storageIntake = storageIntake;
        this.shooterFeed = shooterFeed;
        this.shooter = shooter;
        this.hood = hood;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        hood.setPosition(HoodPosition.HALF);
        shooter.shoot(3500);

        if (shooter.getShooterError()) {

            storageIntake.storageIntakeInSlow();
            shooterFeed.shooterFeedUpSlow();
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