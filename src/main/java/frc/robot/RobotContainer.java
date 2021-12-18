package frc.robot;

import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Swerve.*;
import frc.robot.Utilities.AutoFromTrajectory;
import frc.robot.Utilities.JoystickAnalogButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.AutoDriveLinear;
import frc.robot.Commands.DriveByController;
import frc.robot.Commands.FaceTurret;
import frc.robot.Commands.FeedShooter;
import frc.robot.Commands.FloorIntake;
import frc.robot.Commands.GoalShoot;
import frc.robot.Commands.ShooterDefault;

import frc.robot.Constants.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_robotDrive = new Drivetrain(); //Create Drivetrain Subsystem
  private final Shooter m_shooter = new Shooter(); //Create Shooter Subsystem
  private final Turret m_turret = new Turret(); //Create Turret Subsystem
  private final Intake m_intake = new Intake(); //Create Intake Subsystem

  private final GoalShoot m_goalShoot = new GoalShoot(m_shooter, m_turret, m_robotDrive);   //Create GoalShoot Command
  private final FeedShooter m_feedShoot = new FeedShooter(m_shooter, m_turret,m_intake);    //Create FeedShooter Command
  private final FaceTurret m_faceTurret = new FaceTurret(m_turret, m_robotDrive);           //Create FaceTurret Command
  private final ShooterDefault m_shootDefault = new ShooterDefault(m_shooter);              //Create ShooterDefault Command
  private final FloorIntake m_floorIntake = new FloorIntake(m_intake);
  private final AutoDriveLinear m_testAutoMove = new AutoDriveLinear(m_robotDrive, 3.0, 6.0, 3.25, Math.PI, 5.0,true);

  // The driver's controllers
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final DriveByController m_drive = new DriveByController(m_robotDrive, m_driverController);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings(); // Configure the button bindings to commands using configureButtonBindings function

    // Configure default commands
    m_robotDrive.setDefaultCommand(m_drive); //Set drivetrain default command to "DriveByController" 
    m_turret.setDefaultCommand(m_faceTurret); //Set turret default command to "FaceTurret"
    m_shooter.setDefaultCommand(m_shootDefault); //Set shooter default command to "ShooterDefault"
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
    new POVButton(m_driverController, 180).whenPressed(() -> m_robotDrive.reset(180.0));
    new POVButton(m_driverController, 0).whenPressed(() -> m_robotDrive.reset(0.0));

    // Run "GoalShoot" command when A is pressed on the joystick
    new JoystickButton(m_driverController, Button.kA.value).whenPressed(m_goalShoot);

    // Cancel "GoalShoot" command when A is pressed on the joystick
    new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> m_goalShoot.cancel());

    // Run "FeedShooter" command when X is held down and canel it when button is released
    new JoystickButton(m_driverController, Button.kX.value).whenHeld(m_feedShoot);

    new JoystickButton(m_driverController, Button.kY.value).whenPressed(m_testAutoMove).whenReleased(() -> m_testAutoMove.cancel());

    // Call the changeFieldOrient function when the Right Bumper is pressed
    new JoystickButton(m_driverController, Button.kBumperRight.value).whenPressed(() -> m_drive.changeFieldOrient());
  
    new JoystickAnalogButton(m_driverController, GenericHID.Hand.kRight).whenHeld(m_floorIntake);
  
  }

public Command autoFromTrajectory(String CSV) {
	return AutoFromTrajectory.autoCommand(CSV, m_robotDrive);
}

}