package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeMotor;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.IntakeSolenoidSubsystem;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class IntakeAutoCommand extends CommandBase {
    private IntakeSensors intakeSensors;
    private ShooterFeedSubsytem shooterFeed;
    private StorageIntake storageIntake;
    private IntakeMotor intakeMotor;
    private IntakeSolenoidSubsystem intakeSolenoid;

    public IntakeAutoCommand(IntakeSensors intakeSensors, ShooterFeedSubsytem shooterFeed, StorageIntake storageIntake, IntakeMotor intakeMotor, IntakeSolenoidSubsystem intakeSolenoid) {
        this.intakeSensors = intakeSensors;
        this.shooterFeed = shooterFeed;
        this.storageIntake = storageIntake;
        this.intakeMotor = intakeMotor;
        this.intakeSolenoid = intakeSolenoid;
    }

    public void initialize() {
        // intakeSolenoid.intakeDown();
        intakeMotor.runIntakeIn();
        storageIntake.storageIntakeIn();
        shooterFeed.shooterFeedUp();
    }

    public void execute() {
        if (!intakeSensors.topTrigger()) {
            shooterFeed.shooterFeedStop();

            if (!intakeSensors.bottomTrigger()) {
                storageIntake.storageIntakeStop();
                intakeMotor.stopIntakeIn();
                shooterFeed.shooterFeedStop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        storageIntake.storageIntakeStop();
        intakeMotor.stopIntakeIn();
        // intakeSolenoid.intakeUp();
        shooterFeed.shooterFeedStop();
    }

    public boolean isFinished() {
        return false;
    }
}
