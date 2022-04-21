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


public class MostComplexifiedAuto extends SequentialCommandGroup{
  
    public MostComplexifiedAuto(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner firstMove = new AutoFromPathPlanner(drive, "firstMove", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner mostCompexifiedPath1 = new AutoFromPathPlanner(drive, "MostComplexifiedPath1", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner mostCompexifiedPath2 = new AutoFromPathPlanner(drive, "MostComplexifiedPath2", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner mostCompexifiedPath3 = new AutoFromPathPlanner(drive, "MostComplexifiedPath3", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner mostCompexifiedPath4 = new AutoFromPathPlanner(drive, "MostComplexifiedPath4", Constants.AutoConstants.kMaxSpeed);



        // final AutoFromPathPlanner ComplexerAuto1A = new AutoFromPathPlanner(drive, "ComplexerAuto1A", Constants.AutoConstants.kMaxSpeed);
        // final AutoFromPathPlanner ComplexAuto2 = new AutoFromPathPlanner(drive, "ComplexAuto2", Constants.AutoConstants.kMaxSpeed);


        Command intakeRun = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun2 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun3 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun4 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);


        Command intakeposcCommand = new IntakePosCommand(intakeSolenoid);  
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(firstMove.getInitialPose())),
            intakeposcCommand,
            new ParallelRaceGroup(intakeRun, firstMove),
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem, drive, intakeSensors).withTimeout(1.3),
            new ParallelRaceGroup(intakeRun2, mostCompexifiedPath1),
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem, drive, intakeSensors).withTimeout(1),
            new ParallelRaceGroup(intakeRun3, mostCompexifiedPath2),
            intakeRun4.withTimeout(1.5),
            mostCompexifiedPath3,
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem, drive, intakeSensors).withTimeout(1.3),
            mostCompexifiedPath4
            
            
        );
    }
}