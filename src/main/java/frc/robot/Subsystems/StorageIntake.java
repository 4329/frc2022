package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class StorageIntake extends SubsystemBase {
    private CANSparkMax intakeMotor = new CANSparkMax(Configrun.get(41, "storageIntakeID"), MotorType.kBrushless);

    public StorageIntake() {
        storageIntakeBrake();
        storageIntakeStop();
    }

    public void storageIntakeIn() {
        intakeMotor.set(-Configrun.get(0.5, "storageIntakePower"));
    }

    public void storageIntakeOut() {
        intakeMotor.set(Configrun.get(0.5, "storageIntakePower"));
    }

    public void storageIntakeInSlow() {
        intakeMotor.set(-Configrun.get(0.5, "storageIntakePower") / 2);
    }

    public void storageIntakeOutSlow() {
        intakeMotor.set(Configrun.get(0.5, "storageIntakePower") / 2);
    }

    public void storageIntakeStop() {
        intakeMotor.set(0);
    }

    public void storageIntakeBrake() {
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void storageIntakeCoast() {
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }
    
}
