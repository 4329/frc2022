package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class KISSAuto extends SequentialCommandGroup{

    final AutoFromPathPlanner KISSAuto;

    public KISSAuto(Drivetrain drive) {

    KISSAuto = new AutoFromPathPlanner(drive, "KISSAuto", Constants.AutoConstants.kMaxSpeed);
    addCommands(new InstantCommand(()->drive.resetOdometry(KISSAuto.getInitialPose())),
        KISSAuto
         );

    }
}
