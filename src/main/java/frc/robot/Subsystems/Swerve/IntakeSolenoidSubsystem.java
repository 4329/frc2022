package frc.robot.Subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

import com.fasterxml.jackson.databind.node.NullNode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


public class IntakeSolenoidSubsystem extends SubsystemBase {
    private Solenoid IntakeSolenoid = null;
    boolean intakeUp;

    public IntakeSolenoidSubsystem() {
        intakeUp = Configrun.get(false, "intakeUp");
        System.out.println("going to make new solenoid");
        //IntakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
        System.out.println("i made it");
    }

    public void intakeUp() {
        IntakeSolenoid.set(intakeUp);
        System.out.println("intakeup");
    }

    public void intakeDown() {
        IntakeSolenoid.set(!intakeUp);
        System.out.println("intakedown");
    }
//"hello world"
    public void keepIntakePosition() {
        if (IntakeSolenoid.get() == !intakeUp) {
            intakeDown();
        } else {
            intakeUp();
        }
    }
}
