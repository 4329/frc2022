package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class ShooterFeedSubsytem extends SubsystemBase {

    private TalonSRX frontShooterFeed;
    private final TalonSRX backShooterFeed;

    public ShooterFeedSubsytem() {

        backShooterFeed = new TalonSRX(Configrun.get(42, "BackShooterFeed_ID"));
        if(Configrun.get(0, "FrontShooterFeed_ID") != 0)
        {
            frontShooterFeed = new TalonSRX(Configrun.get(43, "FrontShooterFeed_ID"));
        }
        
        brakeShooterFeed();
    }

    public void shooterFeedUp() {

        if(Configrun.get(0, "FrontShooterFeed_ID") != 0)
        {
        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "ShooterFeedPower"));
        }
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, -Configrun.get(0.5, "ShooterFeedPower"));
    }

    public void shooterFeedUpSlow() {

        if(Configrun.get(0, "FrontShooterFeed_ID") != 0)
        {
        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "ShooterFeedPower") / 2);
        }
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, -Configrun.get(0.5, "ShooterFeedPower") / 2);
    }

    public void shooterFeedDown() {

        if(Configrun.get(0, "FrontShooterFeed_ID") != 0)
        {
        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, -Configrun.get(0.5, "ShooterFeedPower"));
        }
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "ShooterFeedPower"));
    }

    public void shooterFeedStop() {

        if(Configrun.get(0, "FrontShooterFeed_ID") != 0)
        {
        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, 0);
        }
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void brakeShooterFeed() {

        if(Configrun.get(0, "FrontShooterFeed_ID") != 0)
        {
        frontShooterFeed.setNeutralMode(NeutralMode.Brake);
        }
        backShooterFeed.setNeutralMode(NeutralMode.Brake);
    }

    public void coastShooterFeed() {
    
        if(Configrun.get(0, "FrontShooterFeed_ID") != 0)
        {
        frontShooterFeed.setNeutralMode(NeutralMode.Coast);
        }
        backShooterFeed.setNeutralMode(NeutralMode.Coast);
    }

}
