package frc.robot.Subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Configrun;

public class Shooter {

  private PIDController shooterPID;
  private TalonFX shooterwheel1;
  private TalonFX shooterwheel2;
  SimpleMotorFeedforward feedForward;
  private double staticFeedForward = 9001;
  private double velocityFeedForward = 0.5;

  double percent;
  //private NetworkTableEntry percentEntry;
  private NetworkTableEntry pidSetpointErrorEntry;
  private NetworkTableEntry percentEntryNum;
  private NetworkTableEntry pidErrorEntryNum;
  private NetworkTableEntry atSetpoint;
  private NetworkTableEntry belowZero;
  private NetworkTableEntry shooterRPM;
  private NetworkTableEntry manualOverride;


  public Shooter() {

    pidSetpointErrorEntry = Shuffleboard.getTab("shooteryness").add("PID Setpoint Error", 1).withWidget("Graph").withProperties(Map.of("Automatic bounds", false, "Upper bound", 2000, "Lower bound", -500, "Unit", "RPM")).getEntry();
    percentEntryNum = Shuffleboard.getTab("shooteryness").add("percent num", percent).getEntry();
    pidErrorEntryNum = Shuffleboard.getTab("shooteryness").add("PID Error num", 1).getEntry();
    atSetpoint = Shuffleboard.getTab("shooteryness").add("At Setpoint", false).withPosition(0, 1).getEntry();
    belowZero = Shuffleboard.getTab("shooteryness").add("Below Zero", false).withPosition(1, 1).getEntry();
    shooterRPM = Shuffleboard.getTab("shooteryness").add("Shooter RPM", 3500).withPosition(0, 2).getEntry();
    manualOverride = Shuffleboard.getTab("shooteryness").add("Manual Override", true).withPosition(1, 2).getEntry();
    
    shooterPID = new PIDController(
      Configrun.get(2.5, "ShooterP"), 
      Configrun.get(0.0, "ShooterI"),
      Configrun.get(0.0, "ShooterD")
    );
    shooterPID.setTolerance(Configrun.get(100, "ShooterToleranceInRPMs") * 2048.0 / 600.0);
    feedForward = new SimpleMotorFeedforward(12, 0.35, 0.06);

    shooterwheel1 = new TalonFX(Configrun.get(30, "ShooterWheel1ID"));
    shooterwheel2 = new TalonFX(Configrun.get(31, "ShooterWheel2ID"));
    shooterwheel1.setInverted(true);
    shooterwheel2.setInverted(false);
    shooterwheel1.follow(shooterwheel1, FollowerType.PercentOutput);
    shooterwheel1.setNeutralMode(NeutralMode.Coast);
    shooterwheel2.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * @param shooterSetpoint
   */
  public void shoot(double shooterSetpoint) {

    double pidVelocity = shooterwheel2.getSelectedSensorVelocity();
    double setpointCTRE = shooterSetpoint * 2048.0 / 600.0;
    double pidCalculated = shooterPID.calculate(pidVelocity, setpointCTRE);
    pidCalculated += staticFeedForward;
    pidCalculated += pidCalculated * velocityFeedForward;
    // kMaxrpm = 6380;
    // sensor units per rotation = 2048
    // kGearRotation = 1
    // maxPowerCtre = 21,777
    double maxPowerCtre = (6380 / 600) * (2048 / 1);
    percent = pidCalculated / maxPowerCtre;
    shooterwheel2.set(ControlMode.PercentOutput, percent);
  //  percentEntry.setDouble(shooterPID.getSetpoint());
    percentEntryNum.setDouble(shooterPID.getSetpoint() / 2048 * 600);
  }

  public void holdFire() {

    shooterwheel2.set(ControlMode.PercentOutput, 0);
  //  percentEntry.setDouble(shooterPID.getSetpoint());
    percentEntryNum.setDouble(shooterPID.getSetpoint() / 2048 * 600);
  }

  /**
   * @return Whether or not shooterPID is within tolerance
   */
  public boolean getShooterError() {

    pidErrorEntryNum.setDouble(shooterPID.getPositionError() / 2048 * 600);
    pidSetpointErrorEntry.setDouble(shooterPID.getPositionError());
    atSetpoint.setBoolean(shooterPID.atSetpoint());

    if (shooterPID.getPositionError() / 2048 * 600 < 0) {

      belowZero.setBoolean(true);
    }

    return shooterPID.atSetpoint();
  }

  public double manualOverride() {

    if (manualOverride.getBoolean(true)) {

      return shooterRPM.getDouble(3500);
    } else {

      return 3500;
    }
  }

  public void toggleOverride() {

    if (manualOverride.getBoolean(true)) {
      
      manualOverride.setBoolean(false);
    } else {

      manualOverride.setBoolean(true);
    }
  }

}