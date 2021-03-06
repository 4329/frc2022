package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TurretSubsystem;

public class TurretToZeroCommand extends CommandBase{
    private TurretSubsystem turretSubsystem;

    public TurretToZeroCommand(TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        turretSubsystem.turretToZero();
    }
}
