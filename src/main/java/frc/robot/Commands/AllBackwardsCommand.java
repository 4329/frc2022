package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeMotor;
import frc.robot.Subsystems.IntakeSolenoidSubsystem;
//import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;

public class AllBackwardsCommand extends CommandBase {
    private ShooterFeedSubsytem shooterFeed;
    private StorageIntake storageIntake;
    private IntakeMotor intakeMotor;
    private IntakeSolenoidSubsystem intakeSolenoid;

    public AllBackwardsCommand(ShooterFeedSubsytem shooterFeed, StorageIntake storageIntake, IntakeMotor intakeMotor, IntakeSolenoidSubsystem intakeSolenoid) {
        this.intakeSolenoid = intakeSolenoid;
        this.shooterFeed = shooterFeed;
        this.storageIntake = storageIntake;
        this.intakeMotor = intakeMotor;
    }

    public void initialize() {
        intakeSolenoid.intakeDown();
        intakeMotor.runIntakeOut();
        storageIntake.storageIntakeOut();
        shooterFeed.shooterFeedDown();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSolenoid.intakeUp();
        storageIntake.storageIntakeStop();
        intakeMotor.stopIntakeIn();
        shooterFeed.shooterFeedStop();
    }

    public boolean isFinished() {
        return false;
    }
}
