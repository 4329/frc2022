package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import  edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.Swerve.IntakeSolenoidSubsystem;

public class IntakeSolenoidDownCommand extends StartEndCommand {
    public IntakeSolenoidDownCommand(IntakeSolenoidSubsystem intakeSolenoid) {
        super (intakeSolenoid::intakeDown, intakeSolenoid::intakeUp);
          
      
      }     

    
}
