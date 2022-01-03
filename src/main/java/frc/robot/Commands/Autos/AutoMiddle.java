package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.FeedShooter;
import frc.robot.Commands.GoalShoot;
import frc.robot.Constants.*;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class AutoMiddle extends SequentialCommandGroup {
    public AutoMiddle(Drivetrain drive, Intake intake, Shooter shooter, Turret turret){
        final AutoFromPathPlanner initialBackupRev = new AutoFromPathPlanner(drive, "InitialBackupRev", AutoConstants.kMaxSpeed);
        addCommands(
            new InstantCommand(()->drive.resetOdometry(initialBackupRev.getInitialPose())),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(0.20), 
                    initialBackupRev.andThen(new FeedShooter(shooter, turret, intake).withTimeout(2.00))),
                new GoalShoot(shooter, turret, drive)
                    ));
    }
}
