package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class MoveOneMeterAuto extends SequentialCommandGroup{

    final AutoFromPathPlanner moveOneMeter;

    /**
     * Moves the robot one meter forward
     *
     * @param drive
     */
    public MoveOneMeterAuto(Drivetrain drive) {

        moveOneMeter = new AutoFromPathPlanner(drive, "distancepath", Constants.AutoConstants.kMaxSpeed);

        addCommands(new InstantCommand(()->drive.resetOdometry(moveOneMeter.getInitialPose())),
        moveOneMeter
        );
    }
}
