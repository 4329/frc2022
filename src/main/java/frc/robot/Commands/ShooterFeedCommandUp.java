package frc.robot.Commands;

import frc.robot.Subsystems.ShooterFeedSubsytem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ShooterFeedCommandUp extends StartEndCommand {
    public ShooterFeedCommandUp(ShooterFeedSubsytem shooterFeedSubsytem) {

        super(shooterFeedSubsytem::shooterFeedStore, shooterFeedSubsytem::shooterFeedStop,
                shooterFeedSubsytem);
    }
}
