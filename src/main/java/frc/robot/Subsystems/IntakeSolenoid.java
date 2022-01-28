package frc.robot.Subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.*;

public class IntakeSolenoid extends SubsystemBase {
    private Solenoid solenoidintake = new Solenoid(PneumaticsModuleType.REVPH,IntakeConstants.kSolenoidPorts[1]);
    boolean intakeUp;

    public IntakeSolenoid() {
        intakeUp = Configrun.get(false, "intakeUp");
    }

    public void intakeUp() {
        solenoidintake.set(intakeUp);
    }

    public void intakeDown() {
        solenoidintake.set(!intakeUp);
    }

    public void keepIntakePosition() {
        if (solenoidintake.get() == !intakeUp) {
            intakeDown();
        } else {
            intakeUp();
        }
    }
}