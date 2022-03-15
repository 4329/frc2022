package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Climber;

public class ClimberButtonCommandReverse extends CommandBase {
    private XboxController controller;
    private Climber climber;

    public ClimberButtonCommandReverse(XboxController controller, Climber climber) {
        this.controller = controller;
        this.climber = climber;

    }

    public void execute() {
        double climbPower = controller.getLeftTriggerAxis();
        climber.reverseClimb(climbPower * -1);
    }

    public void end(boolean interuppted) {
        climber.stopClimb();

    }

}
