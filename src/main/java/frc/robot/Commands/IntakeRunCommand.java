package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import  edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.Swerve.IntakeMotor;
import frc.robot.Subsystems.Swerve.IntakeSolenoidSubsystem;

public class IntakeRunCommand extends CommandBase{
  private IntakeMotor intakeMotor;
  private IntakeSolenoidSubsystem intakeUp;
    public IntakeRunCommand(IntakeMotor intakeMotor, IntakeSolenoidSubsystem intakeUp) {
       //super (intakeMotor::runIntakeIn, intakeMotor::stopIntakeIn);
      this.intakeMotor = intakeMotor;
      this.intakeUp = intakeUp;

      

    }
    public void initialize() {

      intakeUp.IsIntakeUp();

      if(!intakeUp.IsIntakeUp()){

        intakeMotor.runIntakeIn();

      }
      else{

        intakeMotor.stopIntakeIn();

      }
      
  }

  public void end(boolean interrupted) {
    intakeMotor.stopIntakeIn();

  }
}
