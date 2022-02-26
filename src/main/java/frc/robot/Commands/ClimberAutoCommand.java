//not sure if we need this


package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;

public class ClimberAutoCommand extends CommandBase {
    private Climber climber;
    private static final int TIMETOCLIMB = 2000;


    public ClimberAutoCommand(Climber climber) {
        this.climber = climber;
    }

        public void initialize() {
        climber.engage();
        climber.retract();
    
    }

    public void execute() {
    


    }

    





}
