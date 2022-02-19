package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

//import com.fasterxml.jackson.databind.node.NullNode;

//import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
//import frc.robot.RobotContainer;


public class IntakeSolenoidSubsystem extends SubsystemBase {
    
    
    private static final int SOLENOID_CHANNEL = 0;
    private Solenoid m_Solenoid;
    
    boolean intakeUp;

    public IntakeSolenoidSubsystem(PneumaticHub pneumaticHub) {

        intakeUp = Configrun.get(false, "intakeUp");
        m_Solenoid = pneumaticHub.makeSolenoid(SOLENOID_CHANNEL);
        m_Solenoid.set(Configrun.get(true, "intakeUp"));
    }

    public void intakeUp() {

        intakeUp = true;
        m_Solenoid.set(true);
        System.out.println("intakeup");
    }

    public void intakeDown() {

        m_Solenoid.set(false);
        intakeUp = false;
        System.out.println("intakedown");
    }

    public void changeIntake() {

        if (intakeUp) {

            m_Solenoid.set(false);
            intakeUp = false;
        } else {

            m_Solenoid.set(true);
            intakeUp = true;
        }
    }

    public void toggleIntake() {

        m_Solenoid.toggle();
    }
    
}
