package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Configrun;

public class Climber {
    private Solenoid pivotSolenoid;
    private Solenoid shiftSolenoid;
    private Solenoid extendSolenoid;
    private CANSparkMax climberNeoMotor1;
    private CANSparkMax climberNeoMotor2;
    private CANSparkMax climberNeoMotor3;
    private boolean pivoted = false;
    private boolean shifted = false;


    public Climber (PneumaticHub hubbie) {
        pivotSolenoid = hubbie.makeSolenoid(Configrun.get(1, "pivotSolenoidID"));
        shiftSolenoid = hubbie.makeSolenoid(Configrun.get(2, "shiftSolenoidID"));
        extendSolenoid = hubbie.makeSolenoid(Configrun.get(3, "extendSolenoidID"));
        climberNeoMotor1 = new CANSparkMax (Configrun.get(9,  "climberMotor1ID"), MotorType.kBrushless);
        climberNeoMotor2 = new CANSparkMax (Configrun.get(10,  "climberMotor2ID"), MotorType.kBrushless);
        climberNeoMotor3 = new CANSparkMax (Configrun.get(11,  "climberMotor3ID"), MotorType.kBrushless);
        climberNeoMotor2.follow(climberNeoMotor1);
        climberNeoMotor3.follow(climberNeoMotor1);

    }

    public void pivotClimber() {
        pivotSolenoid.set(true); 
    }
    public void reversePivotClimber(){
        pivotSolenoid.set(false);
    }

    public void shift() {
        shiftSolenoid.set(true);
        shifted = true;
    }
    public void unShift() {
        shiftSolenoid.set(false);
        shifted = false;
    }
    
    public void extend() {
        extendSolenoid.set(true);
        shiftSolenoid.set(true);

    }
    public void retract() {
        extendSolenoid.set(false);
        shiftSolenoid.set(false);

    }
    public void climb() {
        climberNeoMotor1.set(Configrun.get(0.5, "climbPower"));
    }
    public void stopClimb() {
        climberNeoMotor1.set(0);
    }
    public void reverseClimb() {
        climberNeoMotor1.set(Configrun.get(-0.5, "reverseClimbPower"));
    }

    public void togglePivot() {
        if(pivoted) {
            reversePivotClimber();
            pivoted = false;
        }
        else {
            pivotClimber();
            pivoted = true;
        }       
    }

    public void toggleShift() {
        if(shifted) {
            unShift();
        }
        else {
            shift();
        }
    }






    
    // extend - 
    // pivotcommand -- toggle
    // shift + extend  
    // unshift + retract

    // climb == while held
}