package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.Configrun;

public class Shooter
{

  private PIDController shooterPID;
  private TalonFX shooterwheel1;
  private TalonFX shooterwheel2;
  double feedForward = 1;

  double percent;
  private NetworkTableEntry percentOutput;
  private NetworkTableEntry pidError;

  public Shooter (){

    percentOutput = Shuffleboard.getTab("shooteryness").add("percent", percent).withWidget("Graph").getEntry();
    pidError = Shuffleboard.getTab("shooteryness").add("PID Error", 1).withWidget("Graph").getEntry();
    shooterPID = new PIDController(.5, 0, 0);
    shooterPID.setTolerance(2000);
    shooterwheel1 = new TalonFX(Configrun.get(13, "ShooterWheel1ID" ));
    shooterwheel2 = new TalonFX(Configrun.get(14, "ShooterWheel2ID" ));
    shooterwheel1.setInverted(true);
    shooterwheel2.setInverted(true);
    shooterwheel2.follow(shooterwheel1, FollowerType.PercentOutput);
    shooterwheel1.setNeutralMode(NeutralMode.Coast);
    shooterwheel2.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * 
   * @param shooterSetpoint
   */
  public void shoot(double shooterSetpoint) {
        
    double pidVelocity = shooterwheel1.getSelectedSensorVelocity();
    double setpointCTRE = shooterSetpoint * 2048.0 / 600.0;
    double pidCalculated = shooterPID.calculate(pidVelocity, setpointCTRE);
    //kMaxrpm = 6380;
    //sensor units per rotation = 2048
    //kGearRotation = 1
    //maxPowerCtre = 21,777
    double maxPowerCtre = (6380/600) * (2048/1);
    percent = pidCalculated / maxPowerCtre;
    percent = percent + (percent * feedForward);
    shooterwheel1.set(ControlMode.PercentOutput, percent);
    System.out.println("some text " + percent);
    percentOutput.setDouble(shooterPID.getSetpoint());
  }

  public void holdFire() {

    shooterwheel1.set(ControlMode.PercentOutput, 0);
  }
 
  /**
   * @return Whether or not shooterPID is within tolerance
   */
  public boolean getShooterError() {

    pidError.setDouble(shooterPID.getVelocityError());
    return shooterPID.atSetpoint();
  }

}