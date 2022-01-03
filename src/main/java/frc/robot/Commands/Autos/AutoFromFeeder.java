package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.FeedShooter;
import frc.robot.Commands.GoalShoot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class AutoFromFeeder extends SequentialCommandGroup {

    public AutoFromFeeder(Drivetrain drive, Intake intake, Shooter shooter, Turret turret){
        final AutoFromPathPlanner feedToShoot =  new AutoFromPathPlanner(drive, "FeedToShoot", AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner returnToFeed = new AutoFromPathPlanner(drive, "ReturnToFeed", AutoConstants.kMaxSpeed);
        addCommands(
            new InstantCommand(()->drive.resetOdometry(feedToShoot.getInitialPose())),
            new ParallelRaceGroup(
                feedToShoot.andThen(
                    new FeedShooter(shooter, turret, intake).withTimeout(3.50)), 
                new WaitCommand(2.00).andThen( 
                    new GoalShoot(shooter, turret, drive))),
            returnToFeed
            );
    }
    
}
