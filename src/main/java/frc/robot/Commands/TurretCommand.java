package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase{
    
    private TurretSubsystem turretSubsystem;

    public TurretCommand(TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    public void execute() {

        turretSubsystem.targeting();
    }

    public void end() {

        turretSubsystem.turretStop();
    }

    public boolean isFinished() {

        return turretSubsystem.targeted();
    }
}
