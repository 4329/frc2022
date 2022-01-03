package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.FeedShooter;
import frc.robot.Commands.FloorIntake;
import frc.robot.Commands.GoalShoot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class AutoRight extends SequentialCommandGroup {

    public AutoRight(Drivetrain drive, Intake intake, Shooter shooter, Turret turret){
        final AutoFromPathPlanner initialBackup = new AutoFromPathPlanner(drive,"InitialBackup",AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner intakeBalls = new AutoFromPathPlanner(drive, "IntakeBalls",AutoConstants.kMaxSpeed*0.750);
        final AutoFromPathPlanner driveToShoot = new AutoFromPathPlanner(drive, "DriveToShoot",AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner finalMove = new AutoFromPathPlanner(drive, "FinalMove",AutoConstants.kMaxSpeed);

        addCommands( 
            new InstantCommand(()->drive.resetOdometry(initialBackup.getInitialPose())),
            new ParallelCommandGroup(
            new SequentialCommandGroup( 
                new WaitCommand(0.20),

        initialBackup.withTimeout(4.00),

        new FeedShooter(shooter, turret, intake).withTimeout(2.00),

        intakeBalls.raceWith(
            new FloorIntake(intake,false)).withTimeout(4.00), 

            new WaitCommand(0.25),
            
            driveToShoot.withTimeout(4.00),

        new FeedShooter(shooter, turret, intake).withTimeout(3.00),
        
        finalMove),  

        new GoalShoot(shooter, turret, drive)));
    }
    
}
