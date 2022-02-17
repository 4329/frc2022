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
    
    private Solenoid m_Solenoid = null;
    
    boolean intakeUp;

    public IntakeSolenoidSubsystem(PneumaticHub pneumaticHub) {
        intakeUp = Configrun.get(false, "intakeUp");
        System.out.println("going to make new solenoid");
        m_Solenoid = pneumaticHub.makeSolenoid(SOLENOID_CHANNEL);
        System.out.println("i made it");
    }

    public void intakeUp() {
        m_Solenoid.set(intakeUp);
        System.out.println("intakeup");
    }

    public void intakeDown() {
        m_Solenoid.set(!intakeUp);
        System.out.println("intakedown");
    }
//"hello world"
    public void keepIntakePosition() {
        if (m_Solenoid.get() == !intakeUp) {
            intakeDown();
        } else {
            intakeUp();
        }
    }
}
