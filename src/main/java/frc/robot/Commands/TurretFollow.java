package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TrackingTurretSubsystem;
import frc.robot.Subsystems.Swerve.Drivetrain;

public class TurretFollow extends CommandBase {
    private TrackingTurretSubsystem trackingTurretSubsystem;
    private Drivetrain drivetrain;

    public TurretFollow(TrackingTurretSubsystem trackingTurretSubsystem, Drivetrain drivetrain) {
        this.trackingTurretSubsystem = trackingTurretSubsystem;
        this.drivetrain = drivetrain;
        addRequirements(trackingTurretSubsystem);
    }

    @Override
    public void execute() {
        trackingTurretSubsystem.turretFollow(drivetrain.getPose());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
