package frc.robot;

import frc.robot.Subsystems.Swerve.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Autos.IntakeRunAuto;
import frc.robot.Commands.Autos.TwoPathsAuto;
import frc.robot.Commands.Autos.MoveOneMeterAuto;
import frc.robot.Commands.ClimberButtonCommand;
import frc.robot.Commands.ClimberButtonCommandReverse;
import frc.robot.Commands.DriveByController;
import frc.robot.Commands.IntakeBackwardsCommand;
import frc.robot.Commands.IntakePosCommand;
import frc.robot.Commands.ShooterFeedCommandUp;
import frc.robot.Commands.IntakeRunCommand;
import frc.robot.Commands.IntakeSensorsCommand;
import frc.robot.Commands.IntakeSolenoidDownCommand;
import frc.robot.Commands.SensorOutputCommand;
import frc.robot.Commands.StorageIntakeInCommand;
import frc.robot.Commands.StorageIntakeOutCommand;
import frc.robot.Commands.TowerCommand;
import frc.robot.Constants.*;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.Swerve.IntakeMotor;
import frc.robot.Utilities.JoystickAnalogButton;
import edu.wpi.first.wpilibj.PneumaticHub;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.Shooter;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here
 */
public class RobotContainer {

  private final PneumaticHub pneumaticHub;

  // The robot's subsystems
  private final Drivetrain m_robotDrive;
  private final StorageIntake storageIntake;
  private final IntakeSensors intakeSensors;
  private final ShooterFeedSubsytem shooterFeed;
  private final IntakeSolenoidSubsystem intakeSolenoid;
  private final IntakeMotor intakeMotor;
  private final Shooter shooter;
  private final Climber climber;
  // The driver's controllers
  final XboxController m_driverController;
  final XboxController m_operatorController;

  final SendableChooser<Command> m_chooser;

  private final DriveByController m_drive;

  private Command moveOneMeter;
  private Command twoPaths;
  private Command intakeRun;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param drivetrain
   */
  public RobotContainer(Drivetrain drivetrain) {

    pneumaticHub = new PneumaticHub(Configrun.get(61, "PH_CAN_ID"));

    shooter = new Shooter();
    shooterFeed = new ShooterFeedSubsytem();
    storageIntake = new StorageIntake();
    intakeMotor = new IntakeMotor();
    intakeSolenoid = new IntakeSolenoidSubsystem(pneumaticHub);
    intakeSensors = new IntakeSensors();
    climber = new Climber(pneumaticHub);

    initializeCamera();
    
    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    m_robotDrive = drivetrain;
    m_drive = new DriveByController(m_robotDrive, m_driverController);
    m_robotDrive.setDefaultCommand(m_drive); // Set drivetrain default command to "DriveByController"

    configureButtonBindings(); /* Configure the button bindings to commands using configureButtonBindings
                               function */

    m_chooser = new SendableChooser<>();
    configureAutoChooser(drivetrain);
  }

  /** 
   * Creates and establishes camera streams for the shuffleboard ~Ben  
  */
  private void initializeCamera() {

    CameraServer.startAutomaticCapture();
    VideoSource[] enumerateSources = VideoSource.enumerateSources();

    if (enumerateSources.length > 0 && enumerateSources[0].getName().contains("USB")) {
      Shuffleboard.getTab("RobotData").add("Camera 1", enumerateSources[0]).withPosition(5, 0).withSize(4, 4)
          .withWidget(BuiltInWidgets.kCameraStream);
    }
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

  //new JoystickButton(m_driverController, Button.kRightBumper.value).whenPressed(() -> m_drive.changeFieldOrient());

    new JoystickButton(m_operatorController, Button.kY.value).whenPressed(new IntakePosCommand(intakeSolenoid));

    new JoystickButton(m_operatorController, Button.kX.value)
        .whenHeld(new IntakeBackwardsCommand(shooterFeed, storageIntake, intakeMotor, intakeSolenoid));

    new JoystickButton(m_operatorController, Button.kA.value).whileHeld(new ParallelCommandGroup(new IntakeSolenoidDownCommand(intakeSolenoid), new IntakeRunCommand(intakeMotor)));

    // new JoystickButton(m_operatorController, Button.kA.value).whenReleased(new
    // ParallelCommandGroup(intakeStopCommandGroup()));
    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
        .whenHeld(new StorageIntakeInCommand(storageIntake));
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .whenHeld(new TowerCommand(storageIntake, shooterFeed, intakeSensors, shooter));

    new JoystickButton(m_operatorController, Button.kB.value)
        .whenHeld(new IntakeSensorsCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid));
    //new JoystickButton(m_operatorController, Button.kA.value).whenReleased(new ParallelCommandGroup(intakeStopCommandGroup()));
    new JoystickButton(m_operatorController, Button.kLeftBumper.value).whenHeld(new StorageIntakeInCommand(storageIntake));
    new JoystickButton(m_operatorController, Button.kRightBumper.value).whenHeld(new StorageIntakeOutCommand(storageIntake));
    new JoystickButton(m_driverController, Button.kY.value).whenPressed(() -> climber.togglePivot());
    new JoystickButton(m_driverController, Button.kX.value).whenPressed(() -> climber.extend());
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(() -> climber.retract());
    new JoystickAnalogButton(m_operatorController, false).whenHeld(new ClimberButtonCommand(m_operatorController, climber) );
    new JoystickAnalogButton(m_operatorController, true).whenHeld(new ClimberButtonCommandReverse(m_operatorController, climber) );
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> climber.toggleShift());



  }

  /**
   * Pulls autos and configures the chooser
   */
  private void configureAutoChooser(Drivetrain drivetrain) {

    //Pulls autos
    moveOneMeter = new MoveOneMeterAuto(m_robotDrive);
    twoPaths = new TwoPathsAuto(m_robotDrive);
    intakeRun = new IntakeRunAuto(m_robotDrive);

    //Adds autos to the chooser
    m_chooser.setDefaultOption("MoveOneMeterAuto", moveOneMeter);
    m_chooser.addOption("MoveOneMeterAuto", moveOneMeter);
    m_chooser.addOption("TwoPathsAuto", twoPaths);
    m_chooser.addOption("IntakeRunAuto", intakeRun);

    //Puts autos on Shuffleboard
    Shuffleboard.getTab("Autonomous").add("SelectAuto", m_chooser).withSize(2, 1).withPosition(3, 1);
    Shuffleboard.getTab("Autonomous").add("Documentation",
        "Autonomous Modes at https://stem2u.sharepoint.com/sites/frc-4329/_layouts/15/Doc.aspx?sourcedoc={91263377-8ca5-46e1-a764-b9456a3213cf}&action=edit&wd=target%28Creating%20an%20Autonomous%20With%20Pathplanner%7Cb37e1a20-51ec-9d4d-87f9-886aa67fcb57%2F%29")
        .withPosition(2, 2).withSize(4, 1);
  }

  /**
   * @return Selected Auto
   */
  public Command getAuto() {

    return m_chooser.getSelected();
  }

  public void disableRobot() {

    shooterFeed.coastShooterFeed();
    storageIntake.storageIntakeCoast();
  }
}
