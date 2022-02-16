package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class ShooterAutoFireCommand extends CommandBase {
    private ShooterFeedSubsytem shooterFeed;
    private StorageIntake storageIntake;
    private Shooter shooter;

    public ShooterAutoFireCommand(ShooterFeedSubsytem shooterFeed, StorageIntake storageIntake, Shooter shooter) {
        this.shooterFeed = shooterFeed;
        this.storageIntake = storageIntake;
        this.shooter = shooter;
    }

    public void initialize() {
        shooterFeed.shooterFeedUp();
        storageIntake.storageIntakeIn();
        shooter.shoot(0.5);;//don't know what this will do
    }

    public void end() {
        shooterFeed.shooterFeedUp();
        storageIntake.storageIntakeIn();
        shooter.holdFire();
    }

    public boolean isFinished() {
        return false;
    }
}
