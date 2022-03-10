package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.IntakeRunCommand;
import frc.robot.Commands.TowerCommand;
import frc.robot.Subsystems.IntakeMotor;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;


public class ComplexAuto extends SequentialCommandGroup{
  
    public ComplexAuto(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter) {
        
        final AutoFromPathPlanner firstMove = new AutoFromPathPlanner(drive, "ComplexAutoMove1", Constants.AutoConstants.kMaxSpeed);
        Command intakeRun = new IntakeRunCommand(intakeMotor);
        Command towercCommand = new TowerCommand(storageIntake, shooterFeed, shooter);

        addCommands(new InstantCommand(()->drive.resetOdometry(firstMove.getInitialPose())),
        firstMove, intakeRun, towercCommand
        );
    }
}