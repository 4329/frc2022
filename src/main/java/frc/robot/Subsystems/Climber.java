package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Configrun;

public class Climber {
    private static final double MAX_CLIMBER_RANGE = 10000;
    private final DoubleSolenoid pivotSolenoid;
    private final DoubleSolenoid extendSolenoid;
    private final CANSparkMax climberNeoMotor1;
    private final CANSparkMax climberNeoMotor2;
    private RelativeEncoder climbEncoder;
    private boolean pivoted = false;
    private boolean shifted = false;

    private final NetworkTableEntry isPivotedShuffleboard;
    private final NetworkTableEntry isExtendedShuffleboard;
    private final NetworkTableEntry isMoterActiveShuffleboard;
    private NetworkTableEntry winchPositionShuffleboard;
    private PIDController climbMotorPID;
    private double winchPosition;
    private double fullClimbSetpoint;

    public Climber() {

        pivotSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Configrun.get(5, "pivotSolenoidID_1"), Configrun.get(4, "pivotSolenoidID_2"));
        extendSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Configrun.get(6, "extendSolenoidID_1"), Configrun.get(7, "extendSolenoidID_2"));
        climberNeoMotor1 = new CANSparkMax(Configrun.get(9, "climberMotor1ID"), MotorType.kBrushless);
        climberNeoMotor2 = new CANSparkMax(Configrun.get(10, "climberMotor2ID"), MotorType.kBrushless);
        climbEncoder = climberNeoMotor1.getEncoder();
        climberNeoMotor2.follow(climberNeoMotor1);
        climberNeoMotor1.setIdleMode(IdleMode.kBrake);
        climberNeoMotor2.setIdleMode(IdleMode.kBrake);

        isPivotedShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Pivot", false).withPosition(0, 3).getEntry();
        isExtendedShuffleboard = Shuffleboard.getTab("RobotData").add("Extend Climber", false).withPosition(0, 2).getEntry();
        isMoterActiveShuffleboard = Shuffleboard.getTab("RobotData").add("Climber Winch", false).withPosition(1, 2).getEntry();
        winchPositionShuffleboard = Shuffleboard.getTab("RobotData").add("Winch Position", 0).withWidget(BuiltInWidgets.kTextView).withPosition(2, 2).getEntry();
        climbMotorPID = new PIDController(2.2, 0, 0);

    }


    public void pivotClimber() {
        pivotSolenoid.set(Value.kForward);
        isPivotedShuffleboard.setBoolean(true);
    }

    public void reversePivotClimber() {
        pivotSolenoid.set(Value.kReverse);
        isPivotedShuffleboard.setBoolean(false);

    }

    public void climbPidLoop() {
        double output = climbMotorPID.calculate(winchPosition, fullClimbSetpoint);

        output = output / MAX_CLIMBER_RANGE;
    
        climberNeoMotor1.set(output);
    }

    public boolean fullyClimbed() {
        return climbMotorPID.atSetpoint();
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


    public void climberPeriodic() {
        winchPosition = climbEncoder.getPosition();
        winchPositionShuffleboard.setDouble(winchPosition);
    }


}