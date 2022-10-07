package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Commands.AllBackwardsCommand;
import frc.robot.Commands.BumperCommand;
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


public class RejectAutoHigh extends SequentialCommandGroup{
  
    public RejectAutoHigh(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner RejectHigh1 = new AutoFromPathPlanner(drive, "RejectHigh1", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner RejectHigh2 = new AutoFromPathPlanner(drive, "RejectHigh2", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner RejectHigh3 = new AutoFromPathPlanner(drive, "RejectHigh3", Constants.AutoConstants.kMaxSpeed);
      //  final AutoFromPathPlanner RejectHigh4 = new AutoFromPathPlanner(drive, "RejectHigh4", Constants.AutoConstants.kMaxSpeed);


        Command intakeRun = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun2 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun3 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakePosCommand = new IntakePosCommand(intakeSolenoid);
        // Command intakePosCommand2 = new IntakePosCommand(intakeSolenoid);
        // Command intakeReverse = new AllBackwardsCommand(shooterFeed, storageIntake, intakeMotor, intakeSolenoid);



        // Command towercCommand = new TowerCommand(storageIntake, shooterFeed, shooter, hoodSubsystem, turretSubsystem);
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(RejectHigh1.getInitialPose())),
            intakePosCommand,
            new WaitCommand(0.2),
            new ParallelRaceGroup(intakeRun, RejectHigh1),
            new WaitCommand(0.3), 
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem, drive, intakeSensors).withTimeout(2),
            new ParallelRaceGroup(intakeRun2, RejectHigh2),
            intakeRun3.withTimeout(1.5),
            RejectHigh3,
            new BumperCommand(storageIntake, shooterFeed, shooter, hoodSubsystem)
            // RejectHigh4
            );
    }
}