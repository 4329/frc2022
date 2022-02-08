package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class StorageIntake extends SubsystemBase {
    private TalonSRX intakeMotor = new TalonSRX(Configrun.get(41,"storageIntakeID"));

    public void storageIntakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, Configrun.get(0.5,"storageIntakePower"));
    }
    public void storageIntakeOut() {
        intakeMotor.set(ControlMode.PercentOutput, - Configrun.get(0.5,"storageIntakePower"));
    }
    public void storageIntakeStop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}