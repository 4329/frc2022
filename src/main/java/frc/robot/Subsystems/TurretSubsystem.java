package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import frc.robot.Configrun;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TurretSubsystem extends SubsystemBase{
    
    private final TalonSRX turret;

    public TurretSubsystem() {

        turret = new TalonSRX (Configrun.get(41, "turretID"));
    }

    public void turretRight(){
        turret.set(TalonSRXControlMode.PercentOutput, -Configrun.get(0.5, "turretRightPower"));
    }

    public void turretLeft(){
        turret.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "turretLeftPower"));
    }

    public void turretStop(){
        turret.set(TalonSRXControlMode.PercentOutput, Configrun.get(0, "turretStop"));
    }
}

