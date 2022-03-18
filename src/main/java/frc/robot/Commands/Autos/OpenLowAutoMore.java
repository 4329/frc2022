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


public class OpenLowAutoMore extends SequentialCommandGroup{
  
    public OpenLowAutoMore(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner LowAuto1 = new AutoFromPathPlanner(drive, "LowAuto1", Constants.AutoConstants.kMaxSpeed);
        // final AutoFromPathPlanner LowAuto2 = new AutoFromPathPlanner(drive, "LowAuto2", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner OpenLowAutoMore = new AutoFromPathPlanner(drive, "OpenLowAutoMore", Constants.AutoConstants.kMaxSpeed);



        Command intakeRun = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command intakeRun2 = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);

        Command lowshoot = new TowerLowCommand(storageIntake, shooterFeed, shooter, hoodSubsystem);
        CommandGroups groups = new CommandGroups();
        Command intakePosCommand = new IntakePosCommand(intakeSolenoid);
        Command intakePosCommand2 = new IntakePosCommand(intakeSolenoid);


        addCommands(
            new InstantCommand(()->drive.resetOdometry(LowAuto1.getInitialPose())),
            intakePosCommand,
            new ParallelCommandGroup(intakeRun, LowAuto1).withTimeout(5), 
            lowshoot.withTimeout(2.5),
            new ParallelCommandGroup(intakeRun2, OpenLowAutoMore).withTimeout(5),
            intakePosCommand2,
            groups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(2.5)

     
        
         );
    }
}