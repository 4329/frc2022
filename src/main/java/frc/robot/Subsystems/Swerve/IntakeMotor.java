package frc.robot.Subsystems.Swerve;

import frc.robot.Configrun;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class IntakeMotor {

    private TalonSRX intakeMotor = new TalonSRX(Configrun.get(40, "Intake_ID"));
    double intakeSpeed = Configrun.get(0.2,"intakeSpeed");

    public void runIntakeIn()
    {
        intakeMotor.set(ControlMode.PercentOutput, -intakeSpeed);
    }

    /*public void outtake() {
        intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
    }
*/
    public void stopIntakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean isFinished() {
        return false;
    }

}
