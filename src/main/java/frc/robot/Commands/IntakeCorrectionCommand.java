package frc.robot.Commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class IntakeCorrectionCommand extends CommandBase {

    private ShooterFeedSubsytem shooterFeed;
    private StorageIntake storageIntake;
    private int time;

    public IntakeCorrectionCommand(ShooterFeedSubsytem shooterFeed, StorageIntake storageIntake) {

        this.shooterFeed = shooterFeed;
        this.storageIntake = storageIntake;
    }

    public void initialize() {

        time = 0;
    }

    public void execute() {

        time++;

        shooterFeed.shooterFeedDown();
        storageIntake.storageIntakeOut();
    }

    @Override
    public void end(boolean interrupted) {

        storageIntake.storageIntakeStop();
        shooterFeed.shooterFeedStop();
    }

    public boolean isFinished() {

        if (time >= Constants.IntakeConstants.backUpLength) {

            return true;
        } else {

            return false;
        }
    }

}
