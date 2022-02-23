package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Commands.IntakeRunCommand;
import frc.robot.Subsystems.IntakeMotor;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.AutoFromPathPlanner;

public class IntakeRunAuto extends SequentialCommandGroup{

    private IntakeMotor intakeMotor = new IntakeMotor();

    /**
     * Moves the robot one meter forward, 
     * Runs the intake motor
     * Then moves another meter forward
     * 
     * @param drive
     */
    public IntakeRunAuto(Drivetrain drive) {
        
        final AutoFromPathPlanner moveOneMeter = new AutoFromPathPlanner(drive, "distancepath", Constants.AutoConstants.kMaxSpeed);
        final AutoFromPathPlanner moveOneMeter2 = new AutoFromPathPlanner(drive, "distancepath2", Constants.AutoConstants.kMaxSpeed);

        addCommands(
            new InstantCommand(()->drive.resetOdometry(moveOneMeter.getInitialPose())),
            moveOneMeter,
            new IntakeRunCommand(intakeMotor).withTimeout(1),
            moveOneMeter2
        );
    }
}