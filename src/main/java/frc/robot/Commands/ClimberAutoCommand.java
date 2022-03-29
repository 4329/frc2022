//not sure if we need this


package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;

public class ClimberAutoCommand extends CommandBase {
    private Climber climber;


    public ClimberAutoCommand(Climber climber) {
        this.climber = climber;
    }

    public void initialize() {
    

    }

    public void execute() {
         climber.climbPidLoop();    
    

    }

    @Override
    public boolean isFinished() {
        return climber.fullyClimbed();
    }

    

    





}
