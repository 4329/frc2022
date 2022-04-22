package frc.robot.Subsystems;

import java.awt.geom.Point2D;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configrun;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Drivetrain;
import frc.robot.Utilities.LinearInterpolationTable;

public class Shooter {

  private PIDController shooterPID;
  SimpleMotorFeedforward simpleFeedForward;

  private TalonFX shooterwheel1;
  private TalonFX shooterwheel2;

  final Drivetrain drivetrain;

  double percent;
  double pidVelocity;
  double setpointCTRE;
  double pidCalculated;
  double maxPowerCtre;

  private NetworkTableEntry pidSetpointErrorEntry;
  private NetworkTableEntry pidErrorEntryNum;
  private NetworkTableEntry atSetpoint;
  private NetworkTableEntry shooterRPM;
  public NetworkTableEntry manualOverride;
  private NetworkTableEntry aimedSetpoint;
  private NetworkTableEntry ks;
  private NetworkTableEntry kv;
  private NetworkTableEntry update;

  double targetDistance;

  double minDistance;
  double maxDistance;

  private Point2D[] rpmTable = new  Point2D.Double[] {

    new Point2D.Double(67, 2500),
    new Point2D.Double(83, 2600),
    new Point2D.Double(102, 2650),
    new Point2D.Double(120, 2800),
    new Point2D.Double(138, 2950),
    new Point2D.Double(156, 3100),
    new Point2D.Double(174, 3220),
    new Point2D.Double(186, 3225),
    new Point2D.Double(210, 3400),
    new Point2D.Double(225, 3600),
    new Point2D.Double(240, 3650),
    new Point2D.Double(264, 3900),
    new Point2D.Double(276, 3975)
  };
  private LinearInterpolationTable m_rpmTable = new LinearInterpolationTable(rpmTable);

  private Point2D[] hoodTable = new  Point2D.Double[] {

    new Point2D.Double(67, 3),
    new Point2D.Double(83, 5.5),
    new Point2D.Double(102, 11),
    new Point2D.Double(120, 18),
    new Point2D.Double(138, 21),
    new Point2D.Double(156, 24),
    new Point2D.Double(174, 26),
    new Point2D.Double(186, 28),
    new Point2D.Double(210, 28),
    new Point2D.Double(225, 28),
    new Point2D.Double(240, 29),
    new Point2D.Double(264, 30),
    new Point2D.Double(276, 32)



  };
  private LinearInterpolationTable m_hoodTable = new LinearInterpolationTable(hoodTable);






  /**
   * Creates a shooter subsystem
   */
  public Shooter(Drivetrain drivetrain) {

    this.drivetrain = drivetrain;

    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      // Creates a pid setpoint error graph
      pidSetpointErrorEntry = Shuffleboard.getTab("Shooter").add("PID Setpoint Error", 1).withWidget("Graph").withProperties(Map.of("Automatic bounds", false, "Upper bound", 2000, "Lower bound", -500, "Unit", "RPM")).withPosition(2, 0).getEntry();
      // Creates a pid setpoint textbox
      pidErrorEntryNum = Shuffleboard.getTab("Shooter").add("PID Error num", 1).withPosition(1, 0).getEntry();
      // Returns a boolean of whether or not shooter is within tolerance
      atSetpoint = Shuffleboard.getTab("Shooter").add("At Setpoint", false).withPosition(1, 1).getEntry();
      // Input desired RPM whilst manual override is on
      shooterRPM = Shuffleboard.getTab("RobotData").add("Shooter RPM", 3500).withPosition(5, 0).getEntry();
      aimedSetpoint = Shuffleboard.getTab("Limlight").add("Aimed RPM", 1).getEntry();
      ks = Shuffleboard.getTab("RobotData").add("KS", 12).getEntry();
      kv = Shuffleboard.getTab("RobotData").add("KV", 0.35).getEntry();
      update = Shuffleboard.getTab("Shooter").add("Update", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    }
    manualOverride = Shuffleboard.getTab("RobotData").add("Manual Override", false).withPosition(1, 3).withWidget(BuiltInWidgets.kToggleButton).getEntry();

    // Configures PID and feedForward
    shooterPID = new PIDController(0.000005, 0.0003, 0.000);
    shooterPID.setIntegratorRange(-0.0075, 0.0075);

/*     shooterPID = new PIDController(
      Configrun.get(1.5, "ShooterP"),
      Configrun.get(12, "ShooterI"),
      Configrun.get(0.06, "ShooterD")
    ); */
    shooterPID.setTolerance(Constants.ShooterConstants.shooterToleranceInRPMs * 2048.0 / 600.0);
    simpleFeedForward = new SimpleMotorFeedforward(0.0083, 0.0001485);
    /* simpleFeedForward = new SimpleMotorFeedforward(
    ks.getDouble(1),
    kv.getDouble(1)
    ); */
    // Configures the shooter's motors
    shooterwheel1 = new TalonFX(Configrun.get(30, "ShooterWheel1ID"));
    shooterwheel2 = new TalonFX(Configrun.get(31, "ShooterWheel2ID"));
    shooterwheel1.setInverted(true);
    shooterwheel2.setInverted(false);
    shooterwheel1.configVoltageCompSaturation(12.6);
    shooterwheel1.enableVoltageCompensation(true);
    shooterwheel2.configVoltageCompSaturation(12.6);
    shooterwheel2.enableVoltageCompensation(true);
    shooterwheel2.follow(shooterwheel1, FollowerType.PercentOutput);
    shooterwheel1.setNeutralMode(NeutralMode.Coast);
    shooterwheel2.setNeutralMode(NeutralMode.Coast);

    // Configures half hood's edges
    minDistance = 90;
    maxDistance = 204;
  }

  /**
   * Results in the shooter going at desired RPM
   *
   * @param shooterSetpoint
   */
  public void shoot(double shooterSetpoint) {

    //shooterSetpoint is the RPM
    //pidVelocity = shooterwheel1.getSelectedSensorVelocity();
    //setpointCTRE = shooterSetpoint * 2048.0 / 600.0;
    double PIDoutput = shooterPID.calculate(getVelocityRPM(), shooterSetpoint);
    double FFOutput = simpleFeedForward.calculate(shooterSetpoint);
    // kMaxrpm = 6380;
    // sensor units per rotation = 2048
    // kGearRotation = 1
    // maxPowerCtre = 21,777
    //maxPowerCtre = (6380 / 600) * (2048 / 1);
    //percent = pidCalculated / maxPowerCtre;
    shooterwheel1.set(ControlMode.PercentOutput, PIDoutput+FFOutput);
    SmartDashboard.putNumber("Shooter RPM", getVelocityRPM());
  }

  /**
   * Makes the shooter stop shooting
   */
  public void holdFire() {

    shooterwheel1.set(ControlMode.PercentOutput, 0);
  }

  /**
   * @return Whether or not shooterPID is within tolerance
   */
  public boolean getShooterError() {
    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      pidErrorEntryNum.setDouble(shooterPID.getPositionError() / 2048 * 600);
      pidSetpointErrorEntry.setDouble(shooterPID.getPositionError());
      atSetpoint.setBoolean(shooterPID.atSetpoint());
    }
    return shooterPID.atSetpoint();
  }

  /**
   * pulls from the shuffleboard if manual override is enabled
   * and from an aiming equation if manual override is not enabled
   *
   * @param hood
   * @param turret
   * @return shooter RPM
   */
  public double shooterManualOverride(HoodSubsystem hood, TurretSubsystem turret, double targetDistance) {

      if (manualOverride.getBoolean(true)) {

        return shooterRPM.getDouble(3500);
    } else {

      return aim(hood, turret, targetDistance);
    }
  }

  /**
   * If the override is on, it will be turned off and vice versa
   */
  public void toggleOverride() {
      if (manualOverride.getBoolean(true)) {

          manualOverride.setBoolean(false);
        } else {

          manualOverride.setBoolean(true);
        }
    }

  /**
   * Configures the RPM and hood based on distance from the target
   *
   * @param hood
   * @param turret
   * @return aimed shooter RPM
   */
  public double aim(HoodSubsystem hood, TurretSubsystem turret, double targetDistance) {

    double rpmTableValue = m_rpmTable.getOutput(targetDistance);
    double hoodTableValue = m_hoodTable.getOutput(targetDistance);

    hood.setEncoderPosition(hoodTableValue);

    return rpmTableValue;

  }

  public double getVelocityRPM(){
    return shooterwheel1.getSelectedSensorVelocity()*600/2048;
  }

}