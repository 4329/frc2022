package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    private NetworkTableEntry isShiftedShuffleboard;
    private NetworkTableEntry isPivotedShuffleboard;
    private NetworkTableEntry isExtendedShuffleboard;
    private NetworkTableEntry isMoterActiveShuffleboard;




    public Climber (PneumaticHub hubbie) {
        pivotSolenoid = hubbie.makeSolenoid(Configrun.get(1, "pivotSolenoidID"));
        shiftSolenoid = hubbie.makeSolenoid(Configrun.get(2, "shiftSolenoidID"));
        extendSolenoid = hubbie.makeSolenoid(Configrun.get(3, "extendSolenoidID"));
        climberNeoMotor1 = new CANSparkMax (Configrun.get(9,  "climberMotor1ID"), MotorType.kBrushless);
        climberNeoMotor2 = new CANSparkMax (Configrun.get(10,  "climberMotor2ID"), MotorType.kBrushless);
        climberNeoMotor3 = new CANSparkMax (Configrun.get(11,  "climberMotor3ID"), MotorType.kBrushless);
        climberNeoMotor2.follow(climberNeoMotor1);
        climberNeoMotor3.follow(climberNeoMotor1);

        isShiftedShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Shift Active", false).getEntry();
        isPivotedShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Pivot Active", false).getEntry();
        isExtendedShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Extetend Active", false).getEntry();
        isMoterActiveShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Moters Active", false).getEntry();





    }

    public void pivotClimber() {
        pivotSolenoid.set(true); 
        isPivotedShuffleboard.setBoolean(true);
    }
    public void reversePivotClimber(){
        pivotSolenoid.set(false);
        isPivotedShuffleboard.setBoolean(false);

    }

    public void shift() {
        shiftSolenoid.set(true);
        shifted = true;
        isShiftedShuffleboard.setBoolean(true);
    }
    public void unShift() {
        shiftSolenoid.set(false);
        shifted = false;
        isShiftedShuffleboard.setBoolean(false);
    }
    
    public void extend() {
        extendSolenoid.set(true);
        shift();
        isExtendedShuffleboard.setBoolean(true);

    }
    public void retract() {
        extendSolenoid.set(false);
        unShift();
        isExtendedShuffleboard.setBoolean(false);


    }
    public void climb(double climbPower) {
        climberNeoMotor1.set (climbPower);
        isMoterActiveShuffleboard.setBoolean(true);
        System.out.println("Climb Power Is" +climbPower);
    }
    public void stopClimb() {
        climberNeoMotor1.set(0);
        isMoterActiveShuffleboard.setBoolean(false);

    }
    public void reverseClimb() {
        climberNeoMotor1.set(Configrun.get(-0.5, "reverseClimbPower"));
        isMoterActiveShuffleboard.setBoolean(true);

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