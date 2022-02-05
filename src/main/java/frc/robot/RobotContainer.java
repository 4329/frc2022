package frc.robot;

import frc.robot.Subsystems.Swerve.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.Commands.Autos.ExampleAuto;
import frc.robot.Commands.Autos.IntakeRunAuto;
import frc.robot.Commands.Autos.TwoPathsAuto;
import frc.robot.Commands.Autos.MoveOneMeterAuto;
import frc.robot.Commands.DriveByController;
import frc.robot.Commands.ShooterFeedCommandDown;
import frc.robot.Commands.ShooterFeedCommandUp;
import frc.robot.Commands.IntakeRunCommand;
import frc.robot.Commands.IntakeSensorsLogic;
import frc.robot.Commands.IntakeSolenoidDownCommand;
import frc.robot.Commands.StorageIntakeInCommand;
import frc.robot.Commands.StorageIntakeOutCommand;
import frc.robot.Commands.Autos.MoveOneMeterAuto;
import frc.robot.Constants.*;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.Swerve.IntakeMotor;
import edu.wpi.first.wpilibj.PneumaticHub;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here
 */
public class RobotContainer {

private static final int PH_CAN_ID = Configrun.get(61, "PH_CAN_ID");
private PneumaticHub pneumaticHub = new PneumaticHub(PH_CAN_ID);;

private IntakeMotor intakeMotor = new IntakeMotor();
private IntakeSolenoidSubsystem intakeSolenoid = new IntakeSolenoidSubsystem(pneumaticHub);



ParallelCommandGroup intakeCommandGroup() {
  return new ParallelCommandGroup(new IntakeSolenoidDownCommand(intakeSolenoid), new IntakeRunCommand(intakeMotor));
}



// The robot's subsystems
  private final Drivetrain m_robotDrive;
  private final StorageIntake storageIntake;
  private final IntakeSensors intakeSensors;
  private final ShooterFeedSubsytem shooterFeed;
  // The driver's controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final ShooterFeedSubsytem shooterFeedSubsytem = new ShooterFeedSubsytem();

  private final DriveByController m_drive;

  private final Command moveOneMeter;
  private final Command twoPaths;
  private final Command intakeRun;
  //private final Command exampleAuto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * @param drivetrain
   */
  public RobotContainer(Drivetrain drivetrain) {
    m_robotDrive = drivetrain;
    moveOneMeter = new MoveOneMeterAuto(m_robotDrive);
    twoPaths = new TwoPathsAuto(m_robotDrive);
    intakeRun = new IntakeRunAuto(m_robotDrive);
    //exampleAuto = new ExampleAuto(m_robotDrive);
    m_drive = new DriveByController(m_robotDrive, m_driverController);
    storageIntake = new StorageIntake();
    intakeSensors = new IntakeSensors();
    shooterFeed = new ShooterFeedSubsytem();

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

    new JoystickButton(m_operatorController, Button.kY.value).whileHeld(new ShooterFeedCommandUp(shooterFeedSubsytem));
    new JoystickButton(m_operatorController, Button.kX.value).whileHeld(new ShooterFeedCommandDown(shooterFeedSubsytem));

    new JoystickButton(m_operatorController, Button.kA.value).whileHeld(new ParallelCommandGroup(intakeCommandGroup()));

    //new JoystickButton(m_operatorController, Button.kA.value).whenReleased(new ParallelCommandGroup(intakeStopCommandGroup()));
    new JoystickButton(m_operatorController, Button.kLeftBumper.value).whenHeld(new StorageIntakeInCommand(storageIntake));
    new JoystickButton(m_operatorController, Button.kRightBumper.value).whenHeld(new StorageIntakeOutCommand(storageIntake));

    new JoystickButton(m_operatorController, Button.kB.value).whileHeld(new IntakeSensorsLogic(intakeSensors, shooterFeed, storageIntake));
  }

private void configureAutoChooser(){
  m_chooser.addOption("MoveOneMeterAuto", moveOneMeter);
  m_chooser.addOption("TwoPathsAuto", twoPaths);
  m_chooser.addOption("IntakeRunAuto", intakeRun);
  //m_chooser.addOption("ExampleAuto", exampleAuto);
  Shuffleboard.getTab("Autonomous").add("SelectAuto", m_chooser).withSize(2, 1).withPosition(3, 1);
  Shuffleboard.getTab("Autonomous").add("Documentation", "Autonomous Modes at https://stem2u.sharepoint.com/sites/frc-4329/_layouts/15/Doc.aspx?sourcedoc={91263377-8ca5-46e1-a764-b9456a3213cf}&action=edit&wd=target%28Creating%20an%20Autonomous%20With%20Pathplanner%7Cb37e1a20-51ec-9d4d-87f9-886aa67fcb57%2F%29").withPosition(2, 2).withSize(4, 1);
}

  public Command getAuto(){
    return m_chooser.getSelected();
  }

}