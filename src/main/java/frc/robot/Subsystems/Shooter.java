package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Configrun;

public class Shooter
{

  private PIDController shooterPID;
  private TalonFX shooterwheel1;
  private TalonFX shooterwheel2;

  public Shooter (){

    shooterPID = new PIDController(3, 0, 0);
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
    double percent = pidCalculated / maxPowerCtre;   
    shooterwheel1.set(ControlMode.PercentOutput, percent);
  }

  public void holdFire() {

    shooterwheel1.set(ControlMode.PercentOutput, 0);
  }
    
}