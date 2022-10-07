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
import frc.robot.Commands.TowerLowCommand;
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


public class RightThreeBallAuto extends SequentialCommandGroup{
  
    public RightThreeBallAuto(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner ThreeBall1 = new AutoFromPathPlanner(drive, "ThreeBall1", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner ThreeBall2 = new AutoFromPathPlanner(drive, "ThreeBall2", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner ThreeBall3 = new AutoFromPathPlanner(drive, "ThreeBall3", Constants.AutoConstants.kMaxSpeed);



        Command intakeRun = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun2 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakePosCommand = new IntakePosCommand(intakeSolenoid);
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(ThreeBall1.getInitialPose())),
            intakePosCommand,
            new WaitCommand(0.25),
            new ParallelRaceGroup(intakeRun, ThreeBall1),
            new WaitCommand(0.35),
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem, drive, intakeSensors).withTimeout(2),
            new ParallelRaceGroup(intakeRun2, ThreeBall2),
            new WaitCommand(0.35),
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem, drive, intakeSensors).withTimeout(2),
            ThreeBall3

         );
    }
}