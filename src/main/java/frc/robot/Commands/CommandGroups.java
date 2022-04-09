package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Swerve.Drivetrain;

//Put your command groups here

public class CommandGroups {
    public CommandBase fire(TurretSubsystem turretSubsystem, StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hoodSubsystem, Drivetrain drivetrain) {
        return new SequentialCommandGroup(new TurretCommand(turretSubsystem).withTimeout(1), new TowerCommand(storageIntake, shooterFeed, shooter, hoodSubsystem, turretSubsystem, drivetrain).withTimeout(5), new TurretToZeroCommand(turretSubsystem));
    }
    public CommandBase towerLow(StorageIntake storageIntake, ShooterFeedSubsytem shooterFeed, Shooter shooter, HoodSubsystem hoodSubsystem) {
        return new SequentialCommandGroup(new TowerLowCommand(storageIntake, shooterFeed, shooter, hoodSubsystem).withTimeout(5));
    }
}
