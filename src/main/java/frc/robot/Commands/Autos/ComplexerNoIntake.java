package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.CommandGroups;
import frc.robot.Commands.IntakeAutoCommand;
import frc.robot.Commands.IntakePosCommand;
import frc.robot.Commands.IntakeRunCommand;
import frc.robot.Commands.TowerCommand;
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


public class ComplexerNoIntake extends SequentialCommandGroup{
  
    public ComplexerNoIntake(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner firstMove = new AutoFromPathPlanner(drive, "ComplexAutoMove1", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner complexerAuto1 = new AutoFromPathPlanner(drive, "ComplexerAuto1", Constants.AutoConstants.kMaxSpeed);

  
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(firstMove.getInitialPose())),
            firstMove,
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem, drive, intakeSensors).withTimeout(2.5),
            complexerAuto1
            //groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5)

            );
    }
}