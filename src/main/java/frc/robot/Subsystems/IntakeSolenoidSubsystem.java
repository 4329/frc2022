package frc.robot.Subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class IntakeSolenoidSubsystem extends SubsystemBase {

    // private static final int SOLENOID_CHANNEL = 0;
    // private Solenoid m_Solenoid;
    private DoubleSolenoid m_doubleSolenoid = null;

    boolean intakeUp;

    public IntakeSolenoidSubsystem() {

        intakeUp = Configrun.get(false, "intakeUp");
        // m_Solenoid = pneumaticHub.makeSolenoid(SOLENOID_CHANNEL);
        // m_Solenoid.set(Configrun.get(true, "intakeUp"));
        m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Configrun.get(0, "intakeSolenoidID_1"), Configrun.get(1, "intakeSolenoidID_2"));
    }

    public void intakeUp() {

        intakeUp = true;
        // m_Solenoid.set(true);
        m_doubleSolenoid.set(kReverse);
        System.out.println("intakeup");
    }

    public void intakeDown() {

        // m_Solenoid.set(false);
        m_doubleSolenoid.set(kForward);
        intakeUp = false;
        System.out.println("intakedown");
    }

    public void changeIntake() {

        if (intakeUp) {

            // m_Solenoid.set(false);
            m_doubleSolenoid.set(kForward);
            intakeUp = false;
        } else {

            // m_Solenoid.set(true);
            m_doubleSolenoid.set(kReverse);
            intakeUp = true;
        }
    }

    public void toggleIntake() {

        m_doubleSolenoid.toggle();
    }

    // private static final int SOLENOID_CHANNEL = 0;

    // private Solenoid m_Solenoid = null;

    // boolean intakeUp;

}
