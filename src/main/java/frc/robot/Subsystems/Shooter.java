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
import frc.robot.Constants;

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
  private NetworkTableEntry manualOverride;


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
    shooterPID.setTolerance(Constants.ShooterPIDConstants.shooterToleranceInRPMs * 2048.0 / 600.0);
    simpleFeedForward = new SimpleMotorFeedforward(12, 0.35, 0.06);

    shooterwheel1 = new TalonFX(Configrun.get(30, "ShooterWheel1ID"));
    shooterwheel2 = new TalonFX(Configrun.get(31, "ShooterWheel2ID"));
    shooterwheel1.setInverted(true);
    shooterwheel2.setInverted(false);
    shooterwheel2.follow(shooterwheel1, FollowerType.PercentOutput);
    shooterwheel1.setNeutralMode(NeutralMode.Coast);
    shooterwheel2.setNeutralMode(NeutralMode.Coast);
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

  public double manualOverride() {

    if (manualOverride.getBoolean(true)) {

      return shooterRPM.getDouble(3500);
    } else {

      return 3500; //TODO add aiming velocity code here
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