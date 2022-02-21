package frc.robot;

import frc.robot.Subsystems.Turret;
//import frc.robot.Subsystems.EncoderTestSubsystem;
import frc.robot.Subsystems.Swerve.*;
import frc.robot.Utilities.JoystickAnalogButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.DriveByController;
import frc.robot.Commands.GoalShoot;
import frc.robot.Subsystems.Limelight;
//import frc.robot.Commands.EncoderTestMotorBack;
//import frc.robot.Commands.EncoderTestMotorForward;
import frc.robot.Constants.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here
 */
public class RobotContainer {

  private final Turret m_turret = new Turret();
  // The robot's subsystems
  //public final static EncoderTestSubsystem encoderTestSubsystem = new EncoderTestSubsystem();
  private final Drivetrain m_robotDrive;
  private final GoalShoot m_goalShoot = new GoalShoot(m_turret); 

 

  // The driver's controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final DriveByController m_drive;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
 * @param drivetrain
   */
  public RobotContainer(Drivetrain drivetrain) {
    m_robotDrive = drivetrain;
    m_drive = new DriveByController(m_robotDrive, m_driverController);
    initializeCamera();
    configureAutoChooser();
    configureButtonBindings(); // Configure the button bindings to commands using configureButtonBindings
                               // function
    // Configure default commands
    m_robotDrive.setDefaultCommand(m_drive); // Set drivetrain default command to "DriveByController"
  }

  private void initializeCamera() {

  HttpCamera limelight = new HttpCamera("Limelight", "http://10.43.29.11:5800");
  CameraServer.startAutomaticCapture(limelight);

  Shuffleboard.getTab("RobotData").add("Limelight Baby", limelight).withPosition(2, 0).withSize(2, 2)
      .withWidget(BuiltInWidgets.kCameraStream);
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

    new JoystickButton(m_driverController, Button.kA.value).whenPressed(m_goalShoot);
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> m_goalShoot.cancel());


    //new JoystickButton(m_operatorController, Button.kA.value).whileHeld(new EncoderTestMotorForward(encoderTestSubsystem));
    //new JoystickButton(m_operatorController, Button.kB.value).whileHeld(new EncoderTestMotorBack(encoderTestSubsystem));



  }

private void configureAutoChooser(){
  SmartDashboard.putData(m_chooser);
}

public Command getAuto(){
  return m_chooser.getSelected();
}

}