package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.TurretSubsystem;

public class TurretCommandsRight extends StartEndCommand{
    public TurretCommandsRight(TurretSubsystem turretSubsystem) {

        super(turretSubsystem::turretRight, turretSubsystem::turretStop,
                turretSubsystem);
    }
}
