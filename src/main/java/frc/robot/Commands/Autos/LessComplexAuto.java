package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.CommandGroups;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IntakeMotor;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.IntakeSolenoidSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;


public class LessComplexAuto extends SequentialCommandGroup{
  
    public LessComplexAuto(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner firstMove = new AutoFromPathPlanner(drive, "ComplexAutoMove1", Constants.AutoConstants.kMaxSpeed);
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(firstMove.getInitialPose())),
            firstMove.withTimeout(3), 
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5)

         );
    }
}