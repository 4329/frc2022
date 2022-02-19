package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

public class ShooterFeedSubsytem extends SubsystemBase {

    private final TalonSRX frontShooterFeed;
    private final TalonSRX backShooterFeed;

    public ShooterFeedSubsytem(){

        frontShooterFeed = new TalonSRX(Configrun.get(42, "FrontShooterFeed_ID"));
        backShooterFeed = new TalonSRX(Configrun.get(43, "BackShooterFeed_ID"));
        brakeShooterFeed();
    }



    public void shooterFeedUp() {

        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "ShooterFeedPower"));
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, - Configrun.get(0.5, "ShooterFeedPower"));
    }

    public void shooterFeedDown() {

        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, - Configrun.get(0.5, "ShooterFeedPower"));
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, Configrun.get(0.5, "ShooterFeedPower"));
    }

    public void shooterFeedStop() {

        frontShooterFeed.set(TalonSRXControlMode.PercentOutput, 0);
        backShooterFeed.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void brakeShooterFeed(){

        frontShooterFeed.setNeutralMode(NeutralMode.Brake);
        backShooterFeed.setNeutralMode(NeutralMode.Brake);
    }

    public void coastShooterFeed(){

        frontShooterFeed.setNeutralMode(NeutralMode.Coast);
        backShooterFeed.setNeutralMode(NeutralMode.Coast);
    }
    
}
