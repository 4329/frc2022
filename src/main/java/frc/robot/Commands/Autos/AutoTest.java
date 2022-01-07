package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Turret;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class AutoTest extends SequentialCommandGroup {

    public AutoTest(Drivetrain drive, Intake intake, Shooter shooter, Turret turret){
        final AutoFromPathPlanner testAuto = new AutoFromPathPlanner(drive, "Test", AutoConstants.kMaxSpeed);

        addCommands(new InstantCommand(()->drive.resetOdometry(testAuto.getInitialPose())),
            testAuto
        );
    }

}
