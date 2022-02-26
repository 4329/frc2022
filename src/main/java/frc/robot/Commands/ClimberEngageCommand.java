package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;

public class ClimberEngageCommand extends CommandBase{
    private Climber climber;

    public ClimberEngageCommand(Climber climber){
        this.climber = climber;
    }

    public void initialize() {
        //System.currentTimeMillis();

        climber.unShift();//shift or unshift
        climber.extend();
        climber.pivotClimber();
    }


    public void execute() {

    }
}
