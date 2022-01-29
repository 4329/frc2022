package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import  edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.Swerve.IntakeSolenoid;

public class IntakeSolenoidDownCommand extends StartEndCommand {
    public IntakeSolenoidDownCommand(IntakeSolenoid intakeSolenoid) {
        super (intakeSolenoid::intakeDown, intakeSolenoid::intakeUp);
          
      
      }     

    
}
