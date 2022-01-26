package frc.robot;

import frc.robot.Subsystems.Swerve.*;
import frc.robot.Utilities.JoystickAnalogButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.AutoTest;
import frc.robot.Commands.DriveByController;
import frc.robot.Constants.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here
 */
public class RobotContainer {

  // The robot's subsystems
  private final Drivetrain m_robotDrive;

  // The driver's controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final DriveByController m_drive;

  private final Command autoTest;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
 * @param drivetrain
   */
  public RobotContainer(Drivetrain drivetrain) {
    m_robotDrive = drivetrain;
    autoTest = new AutoTest(m_robotDrive);
    m_drive = new DriveByController(m_robotDrive, m_driverController);
    configureAutoChooser();
    configureButtonBindings(); // Configure the button bindings to commands using configureButtonBindings
                               // function
    // Configure default commands
    m_robotDrive.setDefaultCommand(m_drive); // Set drivetrain default command to "DriveByController"
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
   * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
   * {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Reset drivetrain when down/up on the DPad is pressed
    new POVButton(m_driverController, 180)
        .whenPressed(() -> m_robotDrive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI))));
    new POVButton(m_driverController, 0)
        .whenPressed(() -> m_robotDrive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(0.0))));

    new JoystickButton(m_driverController, Button.kRightBumper.value).whenPressed(() -> m_drive.changeFieldOrient());


  }

private void configureAutoChooser(){
  //m_chooser.addOption("AutoTest",autoTest);
  m_chooser.setDefaultOption("autoTest",autoTest);
  SmartDashboard.putData(m_chooser);
}

public Command getAuto(){
  return m_chooser.getSelected();
}

}