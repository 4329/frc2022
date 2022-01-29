package frc.robot.Commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ShooterFeedCommandDown extends StartEndCommand {
    public ShooterFeedCommandDown() {

        super(RobotContainer.shooterFeedSubsytem::shooterFeedDown, RobotContainer.shooterFeedSubsytem::shooterFeedStop,
                RobotContainer.shooterFeedSubsytem);
    }
}