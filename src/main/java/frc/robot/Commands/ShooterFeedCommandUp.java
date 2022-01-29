package frc.robot.Commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ShooterFeedCommandUp extends StartEndCommand {
    public ShooterFeedCommandUp() {

        super(RobotContainer.shooterFeedSubsytem::shooterFeedUp, RobotContainer.shooterFeedSubsytem::shooterFeedStop,
                RobotContainer.shooterFeedSubsytem);
    }
}
