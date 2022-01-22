package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;


public class AutoTest extends SequentialCommandGroup
{
    public AutoTest(Drivetrain drivetrain)
    {
        final AutoFromPathPlanner autoTestPath = new AutoFromPathPlanner(drivetrain,"AutoTest",AutoConstants.kMaxSpeed);

        addCommands(autoTestPath.withTimeout(4.0));
    }

}