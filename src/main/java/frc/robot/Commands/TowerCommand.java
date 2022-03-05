package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class TowerCommand extends CommandBase {

    StorageIntake storageIntake;
    ShooterFeedSubsytem shooterFeed;
    Shooter shooter;
    HoodSubsystem hood;

    double setpoint;

    public TowerCommand(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hood) {

        this.storageIntake = storageIntake;
        this.shooterFeed = shooterFeed;
        this.shooter = shooter;
        this.hood = hood;
    }

    @Override
    public void execute() {

        setpoint = shooter.manualOverride(hood);
        shooter.shoot(setpoint);
        //hood.HoodPeriodic();
        

        if (shooter.getShooterError()) {

            storageIntake.storageIntakeIn();
            shooterFeed.shooterFeedUp();
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
    }

    @Override
    public boolean isFinished() {

        return false;
    }

}
