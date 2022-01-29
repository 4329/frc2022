package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class ShooterFeedSubsytem extends SubsystemBase {
    private TalonSRX FrontShooterFeed = new TalonSRX(Configrun.get(42, "FrontShooterFeed_ID"));
    private TalonSRX BackShooterFeed = new TalonSRX(Configrun.get(43, "BackShooterFeed_ID"));

    public void shooterFeedUp() { 
        FrontShooterFeed.set(TalonSRXControlMode.PercentOutput, 0.5); 
        BackShooterFeed.set(TalonSRXControlMode.PercentOutput, -0.5);
    }

    public void shooterFeedDown() { 
        FrontShooterFeed.set(TalonSRXControlMode.PercentOutput, -0.5); 
        BackShooterFeed.set(TalonSRXControlMode.PercentOutput, 0.5);
    }

    public void shooterFeedStop() { 
        FrontShooterFeed.set(TalonSRXControlMode.PercentOutput, 0); 
        BackShooterFeed.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public boolean isFinished() {
        return false;
    }
}
