// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.LimelightSubsystem;

// public class LimelightCommands extends RunCommand {
//         public LimelightCommands(LimelightSubsystem toRun,
//             Subsystem... requirements);
//     // public LimelightCommands(LimelightSubsystem toRun, RobotContainer ... limelightSubsystem) {
//     //     super(limelightSubsystem.limeLight, RobotContainer.limelightSubsystem);
//     //     //      m_toRun = requireNonNullParam(toRun, "toRun", "RunCommand");
//     //     // addRequirements(LimelightSubsystem);
//     // }
//     // new RunCommand(() -> m_robotDrive.arcadeDrive(
//     // -driverController.getY(GenericHID.Hand.kLeft),
//     // driverController.getX(GenericHID.Hand.kRight)),
//     // m_robotDrive);

//     // public LimelightCommands(Runnable toRun, Subsystem[] limelightSubsystem) {
//     //     super(toRun, limelightSubsystem);
//     // }
// }
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;

public class LimelightCommands extends RunCommand {
    public LimelightCommands() {
        super(RobotContainer.limelightSubsystem::limeLight, RobotContainer.limelightSubsystem);
    }
}