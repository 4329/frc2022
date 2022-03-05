package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;

public class ClimberEngageCommand extends CommandBase{
    private Climber climber;   
    private long initTime;
   // private static final int TIMETOENGAGE = 1750;
    private boolean done = false;

    public ClimberEngageCommand(Climber climber) {
        this.climber = climber;
    }

    public void initialize() {
        //done = false;
        // climber.neutral();//engage or neutral
        climber.extend();
        climber.pivotClimber();
        //initTime =  System.currentTimeMillis();
    }

    // public void execute() {

    //     if (System.currentTimeMillis() - initTime >= TIMETOENGAGE) {
    //         climber.engage();
    //         done = true;
    //     }

    // }
    // @Override
    // public boolean isFinished() {
    //     return done;

    // }
}
