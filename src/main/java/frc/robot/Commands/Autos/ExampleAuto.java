package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class ExampleAuto extends SequentialCommandGroup{

    final AutoFromPathPlanner yourAutoName;

    /**
     * An example auto that doesn't work
     * 
     * @param drive
     */
    public ExampleAuto(Drivetrain drive) {
        
        yourAutoName = new AutoFromPathPlanner(drive, "yourAutoName", Constants.AutoConstants.kMaxSpeed);

        addCommands(new InstantCommand(()->drive.resetOdometry(yourAutoName.getInitialPose())),
        yourAutoName
        );
    }
}