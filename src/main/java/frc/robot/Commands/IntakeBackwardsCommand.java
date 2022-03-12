package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.IntakeMotor;

public class IntakeBackwardsCommand extends CommandBase {
    private IntakeMotor intakeMotor;

    public IntakeBackwardsCommand(IntakeMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
    }

    public void initialize() {

        intakeMotor.runIntakeOut();
    }

    @Override
    public void end(boolean interrupted) {
        intakeMotor.stopIntakeIn();
    }

    public boolean isFinished() {
        return false;
    }
}
