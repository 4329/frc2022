package frc.robot.Commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase{
    
    private TurretSubsystem turretSubsystem;

    public TurretCommand(TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {

        turretSubsystem.targeting();
    }

    @Override
    public void end(boolean interrupted) {
        
        turretSubsystem.turretStop();

    }

    @Override
    public boolean isFinished() {

        return turretSubsystem.targeted();
    }
}
