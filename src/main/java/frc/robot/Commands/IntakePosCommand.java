package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Swerve.IntakeSolenoidSubsystem;

public class IntakePosCommand extends CommandBase {
    
    IntakeSolenoidSubsystem intakeSolenoid;

    public IntakePosCommand(IntakeSolenoidSubsystem intakeSolenoid) {
        this.intakeSolenoid = intakeSolenoid;
    }

    @Override
    public void initialize() {
        
        intakeSolenoid.changeIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
