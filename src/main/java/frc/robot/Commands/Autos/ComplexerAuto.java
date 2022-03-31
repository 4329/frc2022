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


public class ComplexerAuto extends SequentialCommandGroup{
  
    public ComplexerAuto(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner firstMove = new AutoFromPathPlanner(drive, "firstMove", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner ComplexerAuto1 = new AutoFromPathPlanner(drive, "ComplexerAuto1", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner ComplexerAuto1A = new AutoFromPathPlanner(drive, "ComplexerAuto1A", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner ComplexAuto2 = new AutoFromPathPlanner(drive, "ComplexAuto2", Constants.AutoConstants.kMaxSpeed);


        Command intakeRun = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun2 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun3 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);

        Command intakeposcCommand = new IntakePosCommand(intakeSolenoid);  
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(firstMove.getInitialPose())),
            intakeposcCommand,
            new WaitCommand(0.2),
            new ParallelRaceGroup(intakeRun, firstMove),
            new WaitCommand(0.3),
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5),
            new ParallelRaceGroup(intakeRun2, ComplexerAuto1),
            intakeRun3.withTimeout(0.5),
            ComplexerAuto1A,
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5),
            ComplexAuto2
            );
    }
}