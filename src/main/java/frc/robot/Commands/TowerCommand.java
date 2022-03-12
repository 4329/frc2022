package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;

public class TowerCommand extends CommandBase {

    final StorageIntake storageIntake;
    final ShooterFeedSubsytem shooterFeed;
    final Shooter shooter;
    final HoodSubsystem hood;
    final TurretSubsystem turret;

    private double targetDistance;

    double setpoint;

    /**
     * Runs the tower intake and shooter
     * 
     * @param storageIntake
     * @param shooterFeed
     * @param shooter
     * @param hood
     * @param turret
     */
    public TowerCommand(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hood, TurretSubsystem turret) {

        this.storageIntake = storageIntake;
        this.shooterFeed = shooterFeed;
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
    }

    @Override
    public void initialize() {

        targetDistance = turret.getDistanceFromTarget();
    }

    @Override
    public void execute() {

        setpoint = shooter.shooterManualOverride(hood, turret, targetDistance);
        shooter.shoot(setpoint);        

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
        hood.setPosition(HoodPosition.OPEN);
    }

    @Override
    public boolean isFinished() {

        return false;
    }

}
