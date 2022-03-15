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


public class ComplexerAuto extends SequentialCommandGroup{
  
    public ComplexerAuto(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner firstMove = new AutoFromPathPlanner(drive, "ComplexAutoMove1", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner complexerAuto = new AutoFromPathPlanner(drive, "ComplexerAuto", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner complexerAuto1 = new AutoFromPathPlanner(drive, "ComplexerAuto1", Constants.AutoConstants.kMaxSpeed);

        Command intakeRun = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun2 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun3 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakePosCommand = new IntakePosCommand(intakeSolenoid);
        Command intakePosCommand2 = new IntakePosCommand(intakeSolenoid);



        // Command towercCommand = new TowerCommand(storageIntake, shooterFeed, shooter, hoodSubsystem, turretSubsystem);
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(firstMove.getInitialPose())),
            intakePosCommand,
            new ParallelCommandGroup(intakeRun, firstMove).withTimeout(2), 
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5),
            new ParallelCommandGroup(intakeRun2, complexerAuto).withTimeout(2),
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5),
            new ParallelCommandGroup(intakeRun3, complexerAuto1).withTimeout(6),
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5),
            intakePosCommand2
            );
    }
}