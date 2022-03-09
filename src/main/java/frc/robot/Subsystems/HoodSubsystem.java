//Written by Ben Durbin, 2022

package frc.robot.Subsystems;

import java.util.Collections;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class HoodSubsystem extends SubsystemBase {
  private PIDController hoodPID;
  private CANSparkMax hoodwheel;
  private RelativeEncoder hoodEncoder;
  private NetworkTableEntry inputError;
  private int output;
  private NetworkTableEntry sparkOutput;
  private NetworkTableEntry sparkPosition;
  private NetworkTableEntry hoodOverrideIdleMode;
  private NetworkTableEntry overrideSetpointEntry;
  private double setpointDifference;
  private double hoodOpen;
  private double hoodHalf;
  private double hoodClosed;
  private double hoodNeutral;
  private static final int MAX_RANGE = 33;
  private HoodPosition currentPosition = HoodPosition.NEUTRAL;
  private double setpoint;
  private double overrideSetpoint;

  // 29 is the max range the varaible hood can travel without hitting a hard limit
  // or throwing itself off the track

  // Need to define values for HoodMin, HoodMiddle, and HoodFar when we implement
  // the limit switch for zeroing

  public HoodSubsystem() {
    hoodNeutral = Configrun.get(0, "hoodNeutral");
    hoodOpen = Configrun.get(3, "hoodOpen");
    setpoint = hoodOpen;
    overrideSetpoint = 3;
    hoodHalf = Configrun.get(15, "hoodHalf");
    hoodClosed = Configrun.get(30, "hoodClosed");

    hoodwheel = new CANSparkMax(Configrun.get(11, "HoodWheelID"), MotorType.kBrushless);

    hoodEncoder = hoodwheel.getEncoder();
    hoodEncoder.setPosition(0);

    hoodPID = new PIDController(2.2, 0, 0);
    hoodwheel.setIdleMode(IdleMode.kBrake);
    hoodPID.setTolerance(1);

    sparkOutput = Shuffleboard.getTab("Hood Data").add("Spark Output Percent", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Collections.singletonMap("Block Increment", .01)).withPosition(2, 1).getEntry();

    sparkPosition = Shuffleboard.getTab("Hood Data").add("Position", hoodEncoder.getPosition())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1).withSize(1, 1).getEntry();

    hoodOverrideIdleMode = Shuffleboard.getTab("Hood Data").add("Override Idle Mode", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withSize(2, 1).getEntry();

    inputError = Shuffleboard.getTab("Hood Data").add("Check Input", true).withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0).withSize(2, 1)
        .getEntry();
   
    overrideSetpointEntry = Shuffleboard.getTab("Hood Data").add("Setpoint", 3).getEntry();

    Shuffleboard.getTab("Hood Data").add("Read Me", "If functional: Made by Ben Durbin Else: Made by Mr. Emerick")
        .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0).withSize(3, 1);
    
      }

  public void HoodPeriodic() {
    System.out.println("OUTPUT PRE<--------------------------------------" + output);
    double hoodposition = hoodEncoder.getPosition();
    System.out.println("Hood Position" + hoodposition);
    sparkPosition.setDouble(hoodposition);

    // System.out.println("Hood Position<-----" + hoodposition);
    // double setpoint = 0; // sits at neutral position until told otherwise.

    if (NetworkTableInstance.getDefault().getTable("Shooter").getEntry("manualOverride").getBoolean(true)) {

      setpoint = overrideSetpoint; 
    } else {
      if (currentPosition.equals(HoodPosition.OPEN)) {
        setpoint = hoodOpen;
      } else if (currentPosition.equals(HoodPosition.HALF)) {
        setpoint = hoodHalf;
      } else if (currentPosition.equals(HoodPosition.CLOSED)) {
        setpoint = hoodClosed;
      }
  }

    System.out.println("----------" + setpoint);

    double output = hoodPID.calculate(hoodposition, setpoint);

    System.out.println("HOOD PERIODIC <-----------------" + setpoint);
    output = output / MAX_RANGE;
    // setpointDifference = hoodSetpoint.getDouble(0) - hoodposition;

    if (Math.abs(hoodposition) > MAX_RANGE) {
      output = 0;
    }
    hoodwheel.set(output);
    System.out.println("OUTPUT FINAL <-------------" + output);
  }

  // System.out.println(hoodEncoder.getPosition());

  public boolean hoodSet() {
    return hoodPID.atSetpoint();
  }

  public void stop() {
    hoodwheel.set(0);
  }

  public void setPosition(HoodPosition Position) {
    System.out.println("`````````````````````````Setting Position to" + Position + "`````````````````````````");
    // if (Position.equals(HoodPosition.OPEN)) {
    // hoodEncoder.setPosition(hoodOpen);
    // } else if (Position.equals(HoodPosition.HALF)) {
    // hoodEncoder.setPosition(hoodHalf);
    // } else if (Position.equals(HoodPosition.CLOSED)) {
    // hoodEncoder.setPosition(hoodClosed);
    // }


    
    currentPosition = Position;
  }

  public void setEncoderPosition(double position) {

    if (position < 3) {

      overrideSetpoint = 3;
    } else if (position > 33) {

      overrideSetpoint = 33;
    } else {

    overrideSetpoint = position;
    }
  }

  // may be unnecessary
  public void CyclePosition() {
    System.out.println("````````````````````CYCLE POSITION CALLED````````````````````");

    if (currentPosition.equals(HoodPosition.NEUTRAL)) {
      setPosition(HoodPosition.OPEN);
    } else if (currentPosition.equals(HoodPosition.OPEN)) {
      setPosition(HoodPosition.HALF);
    } else if (currentPosition.equals(HoodPosition.HALF)) {
      setPosition(HoodPosition.CLOSED);
    } else if (currentPosition.equals(HoodPosition.CLOSED)) {
      setPosition(HoodPosition.OPEN);
    }
  }

  public enum HoodPosition {
    OPEN, HALF, CLOSED, NEUTRAL;

  }

  public void hoodTestMode() {
    hoodwheel.setIdleMode(IdleMode.kCoast);
    double hoodposition = hoodEncoder.getPosition();
    sparkPosition.setDouble(hoodposition);
  }

  public void hoodOverride() {

    sparkPosition.setDouble(hoodEncoder.getPosition());
    if (NetworkTableInstance.getDefault().getTable("Shooter").getEntry("manualOverride").getBoolean(true)) {

      setEncoderPosition(overrideSetpointEntry.getDouble(3));
      HoodPeriodic();
      if (hoodOverrideIdleMode.getBoolean(true)) {

        hoodwheel.setIdleMode(IdleMode.kBrake);
      } else {

        hoodwheel.setIdleMode(IdleMode.kCoast);

      }
    }

  }

  
}