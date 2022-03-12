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


public class LowAuto extends SequentialCommandGroup{
  
    public LowAuto(Drivetrain drive, IntakeMotor intakeMotor,StorageIntake storageIntake,ShooterFeedSubsytem shooterFeed,Shooter shooter, TurretSubsystem turretSubsystem, HoodSubsystem hoodSubsystem, IntakeSolenoidSubsystem intakeSolenoid, IntakeSensors intakeSensors) {
        
        final AutoFromPathPlanner LowAuto1 = new AutoFromPathPlanner(drive, "LowAuto1", Constants.AutoConstants.kMaxSpeed);
      

        Command intakeRun = new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid);
        Command lowshoot = new TowerLowCommand(storageIntake, shooterFeed, shooter);
        CommandGroups groups = new CommandGroups();

        addCommands(
            new InstantCommand(()->drive.resetOdometry(LowAuto1.getInitialPose())),
            new InstantCommand(()->intakeSolenoid.intakeDown()),
            new ParallelCommandGroup(intakeRun, LowAuto1).withTimeout(2), 
            new InstantCommand(()->intakeSolenoid.intakeUp()),
            lowshoot.withTimeout(3)
     
        
         );
    }
}