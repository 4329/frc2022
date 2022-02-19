package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class TowerCommand extends CommandBase {
    
    StorageIntake storageIntake;
    ShooterFeedSubsytem shooterFeed;
    IntakeSensors intakeSensors;
    Shooter shooter;

    double setpoint;

    public TowerCommand(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, IntakeSensors intakeSensors, Shooter shooter) {

        this.storageIntake = storageIntake;
        this.shooterFeed = shooterFeed;
        this.intakeSensors = intakeSensors;
        this.shooter = shooter;
    }

    @Override
    public void execute() {

        setpoint = 4500;
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
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }

}
