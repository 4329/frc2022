package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.TurretSubsystem;

public class TurretCommandsLeft extends StartEndCommand{
    public TurretCommandsLeft(TurretSubsystem turretSubsystem) {

        super(turretSubsystem::turretLeft, turretSubsystem::turretStop,
                turretSubsystem);
    }
}