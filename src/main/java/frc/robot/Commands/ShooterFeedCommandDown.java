package frc.robot.Commands;

import frc.robot.Subsystems.ShooterFeedSubsytem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class ShooterFeedCommandDown extends StartEndCommand {
    public ShooterFeedCommandDown(ShooterFeedSubsytem shooterFeedSubsytem) {

        super(shooterFeedSubsytem::shooterFeedDown, shooterFeedSubsytem::shooterFeedStop,
                shooterFeedSubsytem);
    }
}