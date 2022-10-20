package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class TowerCommand extends CommandBase {

    final StorageIntake storageIntake;
    final ShooterFeedSubsytem shooterFeed;
    final Shooter shooter;
    final HoodSubsystem hood;
    final TurretSubsystem turret;
    final Drivetrain drivetrain;
    final IntakeSensors intakeSensors;

    private double targetDistance;
    private boolean foundTarget;

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
    public TowerCommand(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hood, TurretSubsystem turret, Drivetrain drivetrain, IntakeSensors intakeSensors) {

        this.storageIntake = storageIntake;
        this.shooterFeed = shooterFeed;
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
        this.drivetrain = drivetrain;
        this.intakeSensors = intakeSensors;
        addRequirements(turret);
        addRequirements(hood);
        //addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        foundTarget = false;
    }

    @Override
    public void execute() {

        if (foundTarget) {

            setpoint = shooter.shooterManualOverride(hood, turret, targetDistance);
            shooter.shoot(setpoint);

            if (shooter.getShooterError()) {

                storageIntake.storageIntakeInSlow();
                shooterFeed.shooterFeedFire();
            } else {

                storageIntake.storageIntakeStop();
                shooterFeed.shooterFeedStop();
            }

        }
        else {
         if (TurretSubsystem.targetVisible()) {

            //drivetrain.lock();
            foundTarget = true;
            targetDistance = TurretSubsystem.getDistanceFromTarget();

         }
        }

        if (intakeSensors.topTrigger() && intakeSensors.bottomTrigger()) {

            //drivetrain.unlock();
        }
    }

    @Override
    public void end(boolean interrupt) {

        storageIntake.storageIntakeStop();
        shooterFeed.shooterFeedStop();
        shooter.holdFire();
        hood.setPosition(HoodPosition.OPEN);
        // drivetrain.unlock();
    }

    @Override
    public boolean isFinished() {

        return false;
    }

}
