package frc.robot.Subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Configrun;
import frc.robot.Constants;
import frc.robot.Subsystems.HoodSubsystem.HoodPosition;

public class Shooter {

  private PIDController shooterPID;
  SimpleMotorFeedforward simpleFeedForward;

  private TalonFX shooterwheel1;
  private TalonFX shooterwheel2;

  double percent;
  double pidVelocity;
  double setpointCTRE;
  double pidCalculated;
  double maxPowerCtre;

  private NetworkTableEntry pidSetpointErrorEntry;
  private NetworkTableEntry pidErrorEntryNum;
  private NetworkTableEntry atSetpoint;
  private NetworkTableEntry belowZero;
  private NetworkTableEntry shooterRPM;
  public NetworkTableEntry manualOverride;

  double targetDistance;

  double minDistance;
  double maxDistance;

  double aMin, bMin, cMin, dMin;

  double aMed, bMed, cMed, dMed;

  double aMax, bMax, cMax, dMax;

  public Shooter() {

    pidSetpointErrorEntry = Shuffleboard.getTab("Shooter").add("PID Setpoint Error", 1).withWidget("Graph").withProperties(Map.of("Automatic bounds", false, "Upper bound", 2000, "Lower bound", -500, "Unit", "RPM")).withPosition(2, 0).getEntry();
    pidErrorEntryNum = Shuffleboard.getTab("Shooter").add("PID Error num", 1).withPosition(1, 0).getEntry();
    atSetpoint = Shuffleboard.getTab("Shooter").add("At Setpoint", false).withPosition(1, 1).getEntry();
    belowZero = Shuffleboard.getTab("Shooter").add("Below Zero", false).withPosition(1, 2).getEntry();
    shooterRPM = Shuffleboard.getTab("Shooter").add("Shooter RPM", 3500).withPosition(5, 0).getEntry();
    manualOverride = Shuffleboard.getTab("Shooter").add("Manual Override", true).withPosition(5, 1).getEntry();

    shooterPID = new PIDController(
      Configrun.get(2.5, "ShooterP"),
      Configrun.get(0.0, "ShooterI"),
      Configrun.get(0.0, "ShooterD")
    );
    shooterPID.setTolerance(Constants.ShooterConstants.shooterToleranceInRPMs * 2048.0 / 600.0);
    simpleFeedForward = new SimpleMotorFeedforward(

    Constants.ShooterPIDConstants.shooterKs,
    Constants.ShooterPIDConstants.shooterKv,
    Constants.ShooterPIDConstants.shooterKa);

    shooterwheel1 = new TalonFX(Configrun.get(30, "ShooterWheel1ID"));
    shooterwheel2 = new TalonFX(Configrun.get(31, "ShooterWheel2ID"));
    shooterwheel1.setInverted(true);
    shooterwheel2.setInverted(false);
    shooterwheel2.follow(shooterwheel1, FollowerType.PercentOutput);
    shooterwheel1.setNeutralMode(NeutralMode.Coast);
    shooterwheel2.setNeutralMode(NeutralMode.Coast);

    minDistance = Constants.ShooterConstants.minDistance;
    maxDistance = Constants.ShooterConstants.maxDistance;

    aMin = Constants.ShooterConstants.aMin;
    bMin = Constants.ShooterConstants.bMin;
    cMin = Constants.ShooterConstants.cMin;
    dMin = Constants.ShooterConstants.dMin;
    
    aMed = Constants.ShooterConstants.aMed;
    bMed = Constants.ShooterConstants.bMed;
    cMed = Constants.ShooterConstants.cMed;
    dMed = Constants.ShooterConstants.dMed;

    aMax = Constants.ShooterConstants.aMax;
    bMax = Constants.ShooterConstants.bMax;
    cMax = Constants.ShooterConstants.cMax;
    dMax = Constants.ShooterConstants.dMax;
  }

  /**
   * @param shooterSetpoint
   */
  public void shoot(double shooterSetpoint) {

    pidVelocity = shooterwheel1.getSelectedSensorVelocity();
    setpointCTRE = shooterSetpoint * 2048.0 / 600.0;
    pidCalculated = shooterPID.calculate(pidVelocity, setpointCTRE);
    pidCalculated += (simpleFeedForward.calculate(shooterPID.getSetpoint()) *
    Constants.ShooterPIDConstants.velocityFeedForwardMultiplier);
    // kMaxrpm = 6380;
    // sensor units per rotation = 2048
    // kGearRotation = 1
    // maxPowerCtre = 21,777
    maxPowerCtre = (6380 / 600) * (2048 / 1);
    percent = pidCalculated / maxPowerCtre;
    shooterwheel1.set(ControlMode.PercentOutput, percent);
  }

  public void holdFire() {

    shooterwheel1.set(ControlMode.PercentOutput, 0);
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

  public double shooterManualOverride(HoodSubsystem hood, TurretSubsystem turret) {

    if (manualOverride.getBoolean(true)) {

      return shooterRPM.getDouble(3500);
    } else {

      return aim(hood, turret);
    }
  }

  public void toggleOverride() {

    if (manualOverride.getBoolean(true)) {

      manualOverride.setBoolean(false);
    } else {

      manualOverride.setBoolean(true);
    }
  }

  public double aim(HoodSubsystem hood, TurretSubsystem turret) { //TODO add aiming code here
    
    targetDistance = turret.getDistanceFromTarget();

    if (targetDistance < minDistance) { // near zone
    
        hood.setPosition(HoodPosition.CLOSED);
        return aMin * Math.pow(targetDistance, 3) + bMin * Math.pow(targetDistance, 2) + cMin * targetDistance + dMin;
    } else if (minDistance <= targetDistance && targetDistance <= maxDistance) { // middle zone

        hood.setPosition(HoodPosition.HALF);
        return aMed * Math.pow(targetDistance, 3) + bMed * Math.pow(targetDistance, 2) + cMed * targetDistance + dMed;
    } else if (maxDistance < targetDistance) { // far zone
  
        hood.setPosition(HoodPosition.OPEN);
        return aMax * Math.pow(targetDistance, 3) + bMax * Math.pow(targetDistance, 2) + cMax * targetDistance + dMax;
    } else {

      return 3500;
    }
  }

}