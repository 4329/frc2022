package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;

public class ClimberButtonCommand extends CommandBase {
    private XboxController controller;
    private Climber climber;

    public ClimberButtonCommand(XboxController controller, Climber climber) {
        this.controller = controller;
        this.climber = climber;

    }

    public void execute() {
            double climbPower = controller.getRightTriggerAxis();
            climber.climb(climbPower);
            System.out.println("Climbing with" +climbPower);
    }
    
    public void end(boolean interuppted) {
            climber.stopClimb();
            System.out.println("end climb");
    }
}
