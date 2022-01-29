package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class ShooterFeedSubsytem extends SubsystemBase {

    public ShooterFeedSubsytem(){
        brakeShooterFeed();
    } 

    private TalonSRX frontShooterFeed = new TalonSRX(Configrun.get(42, "FrontShooterFeed_ID"));
    private TalonSRX backShooterFeed = new TalonSRX(Configrun.get(43, "BackShooterFeed_ID"));

    public void shooterFeedUp() { 
        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "FrontShooterFeedUp")); 
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, -Configrun.get(0.5, "BackShooterFeedUp"));
    }

    public void shooterFeedDown() { 
        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, -Configrun.get(0.5, "FrontShooterFeedDown")); 
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "BackShooterFeedDown"));
    }

    public void shooterFeedStop() { 
        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0, "FrontShooterFeedStop")); 
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0, "BackShooterFeedStop"));
    }

    public void brakeShooterFeed(){
        frontShooterFeed.setNeutralMode(NeutralMode.Brake);
        backShooterFeed.setNeutralMode(NeutralMode.Brake);
    }    
}
