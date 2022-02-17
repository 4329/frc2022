package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import  edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.Swerve.IntakeMotor;

public class IntakeRunCommand extends StartEndCommand{
    public IntakeRunCommand(IntakeMotor intakeMotor) {
      super (intakeMotor::runIntakeIn, intakeMotor::stopIntakeIn);

    }
}
