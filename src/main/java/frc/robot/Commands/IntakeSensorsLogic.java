package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.Swerve.IntakeMotor;
import frc.robot.Subsystems.Swerve.IntakeSolenoidSubsystem;

public class IntakeSensorsLogic extends CommandBase {
    private IntakeSensors intakeSensors;
    private ShooterFeedSubsytem shooterFeed;
    private StorageIntake storageIntake;
    private IntakeMotor intakeMotor;
    private IntakeSolenoidSubsystem intakeSolenoid;

    public IntakeSensorsLogic(IntakeSensors intakeSensors, ShooterFeedSubsytem shooterFeed, StorageIntake storageIntake) {
        this.intakeSensors = intakeSensors;
        this.shooterFeed = shooterFeed;
        this.storageIntake = storageIntake;
    }

    public void initialize() {
        intakeSolenoid.intakeDown();
        intakeMotor.runIntakeIn();
        storageIntake.storageIntakeIn();
        shooterFeed.shooterFeedUp();
    }

    public void execute() {
        if (intakeSensors.topTrigger()) {
            System.out.println("top deactivated");
            shooterFeed.shooterFeedStop();

            if (intakeSensors.bottomTrigger()) {
                System.out.println("bottom deactivated");
                storageIntake.storageIntakeStop();
                intakeMotor.stopIntakeIn();
                intakeSolenoid.intakeUp();
            }
        }
    }

    public boolean isFinished() {
        return true;
    }
}
