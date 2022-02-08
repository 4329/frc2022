package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class TwoPathsAuto extends SequentialCommandGroup{

    /**
     * Moves the robot one meter forward, 
     * waits three seconds,
     * then moves another meter forward
     * 
     * @param drive
     */
    public TwoPathsAuto(Drivetrain drive) {
        
        final AutoFromPathPlanner moveOneMeter = new AutoFromPathPlanner(drive, "distancepath", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner moveOneMeter2 = new AutoFromPathPlanner(drive, "distancepath2", Constants.AutoConstants.kMaxSpeed);

        addCommands(
            new InstantCommand(()->drive.resetOdometry(moveOneMeter.getInitialPose())),
            moveOneMeter,
            new WaitCommand(3),
            moveOneMeter2
        );
    }
}