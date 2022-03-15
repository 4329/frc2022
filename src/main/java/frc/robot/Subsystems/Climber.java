package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Configrun;

public class Climber {
    private final DoubleSolenoid pivotSolenoid;
    private final DoubleSolenoid shiftSolenoid;
    private final DoubleSolenoid extendSolenoid;
    private final CANSparkMax climberNeoMotor1;
    private final CANSparkMax climberNeoMotor2;
    //private final CANSparkMax climberNeoMotor3;
    private boolean pivoted = false;
    private boolean shifted = false;

    //rivate final NetworkTableEntry isShiftedShuffleboard;
    private final NetworkTableEntry isPivotedShuffleboard;
    private final NetworkTableEntry isExtendedShuffleboard;
    private final NetworkTableEntry isMoterActiveShuffleboard;

    public Climber(PneumaticHub hubbie) {

        pivotSolenoid = hubbie.makeDoubleSolenoid(Configrun.get(5, "pivotSolenoidID_1"), Configrun.get(4, "pivotSolenoidID_2"));
        shiftSolenoid = hubbie.makeDoubleSolenoid(Configrun.get(2, "shiftSolenoidID_1"), Configrun.get(3, "shiftSolenoidID_2"));
        extendSolenoid = hubbie.makeDoubleSolenoid(Configrun.get(6, "extendSolenoidID_1"), Configrun.get(7, "extendSolenoidID_2"));
        climberNeoMotor1 = new CANSparkMax(Configrun.get(9, "climberMotor1ID"), MotorType.kBrushless);
        climberNeoMotor2 = new CANSparkMax(Configrun.get(10, "climberMotor2ID"), MotorType.kBrushless);
        // climberNeoMotor3 = new CANSparkMax(Configrun.get(11, "climberMotor3ID"), MotorType.kBrushless);
        climberNeoMotor2.follow(climberNeoMotor1);
        //climberNeoMotor3.follow(climberNeoMotor1);
        climberNeoMotor1.setIdleMode(IdleMode.kBrake);
        climberNeoMotor2.setIdleMode(IdleMode.kBrake);

        //isShiftedShuffleboard = Shuffleboard.getTab("ClimberData").add("Climber Winch in Gear", false).getEntry();
        isPivotedShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Pivot", false).withPosition(0, 3).getEntry();
        isExtendedShuffleboard = Shuffleboard.getTab("RobotData").add("Extend Climber", false).withPosition(0, 2).getEntry();
        isMoterActiveShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Winch", false).withPosition(1, 2).getEntry();
    }


    public void pivotClimber() {
        pivotSolenoid.set(Value.kForward);
        isPivotedShuffleboard.setBoolean(true);
    }

    public void reversePivotClimber() {
        pivotSolenoid.set(Value.kReverse);
        isPivotedShuffleboard.setBoolean(false);

    }

    public void engage() {//neutral or engage

        shiftSolenoid.set(Value.kForward);
        shifted = true;
        //isShiftedShuffleboard.setBoolean(true);
    }

    public void neutral() {//neutral or engage

        shiftSolenoid.set(Value.kReverse);
        shifted = false;
        //isShiftedShuffleboard.setBoolean(false);
    }

    public void extend() {

        extendSolenoid.set(Value.kForward);
        isExtendedShuffleboard.setBoolean(true);

    }

    public void retract() {

        extendSolenoid.set(Value.kReverse);
        isExtendedShuffleboard.setBoolean(false);
    }

    public void climb(double climbPower) {
        System.out.println("ClimbPower is " + climbPower);
        if (Math.abs(climbPower) > 0.1 && extendSolenoid.get().equals(Value.kForward)) {
            retract();
        }

        climberNeoMotor1.set(climbPower);
        isMoterActiveShuffleboard.setBoolean(true);


    }

    public void stopClimb() {

        climberNeoMotor1.set(0);
        isMoterActiveShuffleboard.setBoolean(false);
    }

    public void reverseClimb(double climbPower) {
        System.out.println("ReverseClimbPower is " + climbPower);
        //should we add extend or retract?!
        climberNeoMotor1.set(climbPower);
        isMoterActiveShuffleboard.setBoolean(true);
    }

    public void togglePivot() {
        if (pivoted) {

            reversePivotClimber();
            pivoted = false;
        } else {

            pivotClimber();
            pivoted = true;
        }
    }

    public void toggleShift() {

        if (shifted) {

            neutral();
        } else {

            engage();
        }
    }

    // extend -
    // pivotcommand -- toggle
    // shift + extend
    // unshift + retract

    // climb == while held
}