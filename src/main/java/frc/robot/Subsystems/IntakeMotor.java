package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Configrun;

public class IntakeMotor {

    private CANSparkMax intakeMotor = new CANSparkMax(Configrun.get(40, "Intake_ID"), MotorType.kBrushless);
    double intakeSpeed = Configrun.get(0.2, "intakeSpeed");

    public void runIntakeIn() {
        intakeMotor.set(intakeSpeed);
    }

    public void runIntakeOut() {
        intakeMotor.set(-intakeSpeed);
    }

    public void stopIntakeIn() {
        intakeMotor.set(0);
    }

    public boolean isFinished() {
        return false;
    }

}
