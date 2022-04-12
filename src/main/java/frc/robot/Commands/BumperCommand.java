package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;

public class BumperCommand extends CommandBase {

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
    public BumperCommand(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hood) {

        this.storageIntake = storageIntake;
        this.shooterFeed = shooterFeed;
        this.shooter = shooter;
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {

        storageIntake.storageIntakeIn();
        shooterFeed.shooterFeedStore();
    }

    @Override
    public void execute() {

        hood.setPosition(HoodPosition.OPEN);
        shooter.shoot(950);
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
