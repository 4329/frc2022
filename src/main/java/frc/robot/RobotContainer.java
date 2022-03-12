package frc.robot;

import javax.print.attribute.HashPrintJobAttributeSet;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.ClimberButtonCommand;
import frc.robot.Commands.ClimberButtonCommandReverse;
import frc.robot.Commands.ClimberEngageCommand;
import frc.robot.Commands.CommandGroups;
import frc.robot.Commands.DriveByController;
import frc.robot.Commands.IntakeAutoCommand;
import frc.robot.Commands.IntakeBackwardsCommand;
import frc.robot.Commands.AllBackwardsCommand;
import frc.robot.Commands.IntakePosCommand;
import frc.robot.Commands.IntakeRunCommand;
import frc.robot.Commands.ManualHoodCommand;
import frc.robot.Commands.SensorOutputCommand;
import frc.robot.Commands.TowerCommand;
import frc.robot.Commands.TurretCommand;
import frc.robot.Commands.TurretToZeroCommand;
import frc.robot.Commands.Autos.IntakeRunAuto;
import frc.robot.Commands.Autos.MoveOneMeterAuto;
import frc.robot.Commands.Autos.TwoPathsAuto;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IntakeMotor;
import frc.robot.Subsystems.IntakeSensors;
import frc.robot.Subsystems.IntakeSolenoidSubsystem;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterFeedSubsytem;
import frc.robot.Subsystems.StorageIntake;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.JoystickAnalogButton;
import frc.robot.Commands.TowerLowCommand;

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
  private final TurretSubsystem turretSubsystem;
  private final IntakeSolenoidSubsystem intakeSolenoid;
  private final IntakeMotor intakeMotor;
  private final Shooter shooter;
  private final Climber climber;
  private final CommandGroups commandGroups;
  private final HoodSubsystem hoodSubsystem;
  // The driver's controllers
  final XboxController m_driverController;
  final XboxController m_operatorController;

  final SendableChooser<Command> m_chooser;

  private final DriveByController m_drive;

  private Command moveOneMeter;
  private Command twoPaths;
  private Command intakeRun;

  private final SensorOutputCommand sensorOutputCommand;
  private final TurretCommand turretCommand;
  private final TurretToZeroCommand turretToZeroCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   * @param drivetrain
   */
  public RobotContainer(Drivetrain drivetrain) {
    m_robotDrive = drivetrain;

    pneumaticHub = new PneumaticHub(Configrun.get(61, "PH_CAN_ID"));

    turretSubsystem = new TurretSubsystem();
    shooter = new Shooter();
    shooterFeed = new ShooterFeedSubsytem();
    storageIntake = new StorageIntake();
    intakeMotor = new IntakeMotor();
    intakeSolenoid = new IntakeSolenoidSubsystem(pneumaticHub);
    intakeSensors = new IntakeSensors();
    climber = new Climber(pneumaticHub);
    sensorOutputCommand = new SensorOutputCommand(intakeSensors);
    intakeSensors.setDefaultCommand(sensorOutputCommand);
    hoodSubsystem = new HoodSubsystem();

    turretCommand = new TurretCommand(turretSubsystem);
    turretToZeroCommand = new TurretToZeroCommand(turretSubsystem);
    commandGroups = new CommandGroups();


    initializeCamera();

    m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    m_drive = new DriveByController(m_robotDrive, m_driverController);
    m_robotDrive.setDefaultCommand(m_drive); // Set drivetrain default command to "DriveByController"

    configureButtonBindings(); /*
                                * Configure the button bindings to commands using configureButtonBindings
                                * function
                                */

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
      Shuffleboard.getTab("RobotData").add("Camera", enumerateSources[0]).withPosition(5, 0).withSize(3, 3)
          .withWidget(BuiltInWidgets.kCameraStream);
    }
    HttpCamera limelight = new HttpCamera("Limelight", "http://10.43.29.11:5800");
    CameraServer.startAutomaticCapture(limelight);

    Shuffleboard.getTab("RobotData").add("Limelight Camera", limelight).withPosition(2, 0).withSize(2, 2)
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
      //Driving
    new POVButton(m_driverController, 180).whenPressed(() -> m_robotDrive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(Math.PI))));// Reset drivetrain when down/up on the DPad is pressed; TODO: what does this do
    new POVButton(m_driverController, 0).whenPressed(() -> m_robotDrive.resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(0.0))));//TODO: what does this do
    new JoystickButton(m_driverController, Button.kRightBumper.value).whenPressed(() -> m_drive.changeFieldOrient());//toggle field dorientation
      //Climber arm controls
    new JoystickButton(m_driverController, Button.kY.value).whenPressed(() -> climber.togglePivot());
    new JoystickButton(m_driverController, Button.kX.value).whenPressed(() -> climber.extend());
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(() -> climber.retract());
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> climber.toggleShift());
      //Climber motor controls
    new JoystickAnalogButton(m_driverController, false).whenHeld(new ClimberButtonCommand(m_driverController, climber));//climb up
    new JoystickAnalogButton(m_driverController, true).whenHeld(new ClimberButtonCommandReverse(m_driverController, climber));//climb down
    new JoystickButton(m_driverController, Button.kLeftBumper.value).whenPressed(new ClimberEngageCommand(climber));//extend & pivot arms
      //Developer tools TODO
    new JoystickButton(m_driverController, Button.kStart.value).whileHeld(new ManualHoodCommand(hoodSubsystem, shooter));
    new JoystickButton(m_driverController, Button.kBack.value).whenPressed(() -> hoodSubsystem.CyclePosition());


    //Operator Controller
      //Shoot
    new JoystickButton(m_operatorController, Button.kY.value).whenHeld(commandGroups.fire(turretSubsystem, storageIntake, shooterFeed, shooter, hoodSubsystem));//shoot high with aimbot
    new JoystickButton(m_operatorController, Button.kBack.value).whenHeld(new TowerCommand(storageIntake, shooterFeed, shooter, hoodSubsystem, turretSubsystem));//shoot high without aimbot
    new JoystickButton(m_operatorController, Button.kA.value).whenHeld(commandGroups.towerLow(storageIntake, shooterFeed, shooter, hoodSubsystem));//shoot low
      //Manage cargo
    new JoystickButton(m_operatorController, Button.kX.value).whenPressed(new IntakePosCommand(intakeSolenoid));//intake up/down
    new JoystickButton(m_operatorController, Button.kB.value).whenHeld(new IntakeAutoCommand(intakeSensors, shooterFeed, storageIntake, intakeMotor, intakeSolenoid));//store
    new JoystickButton(m_operatorController, Button.kRightBumper.value).whenHeld(new AllBackwardsCommand(shooterFeed, storageIntake, intakeMotor));//eject
    new JoystickButton(m_operatorController, Button.kLeftBumper.value).whenHeld(new IntakeBackwardsCommand(intakeMotor));
  }

  /**
   * Pulls autos and configures the chooser
   */
  private void configureAutoChooser(Drivetrain drivetrain) {

    // Pulls autos
    moveOneMeter = new MoveOneMeterAuto(m_robotDrive);
    twoPaths = new TwoPathsAuto(m_robotDrive);
    intakeRun = new IntakeRunAuto(m_robotDrive);

    // Adds autos to the chooser
    m_chooser.setDefaultOption("MoveOneMeterAuto", moveOneMeter);
    m_chooser.addOption("MoveOneMeterAuto", moveOneMeter);
    m_chooser.addOption("TwoPathsAuto", twoPaths);
    m_chooser.addOption("IntakeRunAuto", intakeRun);

    // Puts autos on Shuffleboard
    Shuffleboard.getTab("RobotData").add("SelectAuto", m_chooser).withSize(2, 1).withPosition(0, 0);
    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      Shuffleboard.getTab("Autonomous").add("Documentation",
          "Autonomous Modes at https://stem2u.sharepoint.com/sites/frc-4329/_layouts/15/Doc.aspx?sourcedoc={91263377-8ca5-46e1-a764-b9456a3213cf}&action=edit&wd=target%28Creating%20an%20Autonomous%20With%20Pathplanner%7Cb37e1a20-51ec-9d4d-87f9-886aa67fcb57%2F%29")
          .withPosition(2, 2).withSize(4, 1);
    }
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

  public void init() {

    turretSubsystem.setDefaultCommand(turretToZeroCommand);
    climber.engage();
  }

  public void teleopPeriodic() {
    turretSubsystem.putValuesToShuffleboard();
    hoodSubsystem.hoodOverride(shooter);
  }

  public void test() {

    hoodSubsystem.hoodTestMode();
    turretSubsystem.putValuesToShuffleboard();
  }

}