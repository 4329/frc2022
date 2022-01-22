package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class SimpleAuto extends SequentialCommandGroup {

    public SimpleAuto(Drivetrain drive){
        final AutoFromPathPlanner testAuto = new AutoFromPathPlanner(drive, "Test", AutoConstants.kMaxSpeed);

        addCommands(new InstantCommand(()->drive.resetOdometry(testAuto.getInitialPose())),
            testAuto
        );
    }

}