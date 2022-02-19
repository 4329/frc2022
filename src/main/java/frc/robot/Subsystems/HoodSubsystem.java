//Written by Ben Durbin, 2022

package frc.robot.Subsystems;

import java.util.Collections;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  private NetworkTableEntry hoodSetpoint;
  private double setpointDifference;
  private double hoodClose;
  private double hoodMiddle;
  private double hoodFar;
  private static final int MAX_RANGE = 29;
  // 29 is the max range the varaible hood can travel without hitting a hard limit
  // or throwing itself off the track

  // Need to define values for HoodMin, HoodMiddle, and HoodFar when we implement
  // the limit switch for zeroing

  public HoodSubsystem() {

    hoodwheel = new CANSparkMax(Configrun.get(1, "HoodWheelID"), MotorType.kBrushless);

    hoodEncoder = hoodwheel.getEncoder();

    hoodPID = new PIDController(1.25, 0, 0);
    hoodwheel.setIdleMode(IdleMode.kBrake);
    // hoodPID.setTolerance(1);

    sparkOutput = Shuffleboard.getTab("Hood Data").add("Spark Output Percent", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Collections.singletonMap("Block Increment", .01)).withPosition(2, 1).getEntry();

    sparkPosition = Shuffleboard.getTab("Hood Data").add("Position", hoodEncoder.getPosition())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 1).withSize(2, 1).getEntry();

    hoodSetpoint = Shuffleboard.getTab("Hood Data").add("Set Point", hoodEncoder.getPosition())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(1, 0).withSize(1, 1).getEntry();

    inputError = Shuffleboard.getTab("Hood Data").add("Check Input", true).withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0)
        .getEntry();

    Shuffleboard.getTab("Hood Data").add("Read Me", "If functional: Made by Ben Durbin Else: Made by Mr. Emerick")
        .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0).withSize(3, 1);

    DetermineHoodProfiles();
  }

  public void HoodPeriodic() {
    double hoodposition = hoodEncoder.getPosition();
    sparkPosition.setDouble(hoodposition);

    double output = hoodPID.calculate(hoodposition, hoodSetpoint.getDouble(0));
    output = output / MAX_RANGE;
    setpointDifference = hoodSetpoint.getDouble(0) - hoodposition;

    if (Math.abs(setpointDifference) > MAX_RANGE) {
      output = hoodposition;
      inputError.setBoolean(false);
    }
    hoodwheel.set(output);
  }

  private void DetermineHoodProfiles() {
    hoodClose = hoodEncoder.getPosition();
    hoodMiddle = hoodEncoder.getPosition() + MAX_RANGE / 2;
    hoodFar = hoodEncoder.getPosition() + 26; // Adding 26 instead of the max 29 incase the manual zero doesn't get it
                                              // to true 0
  }

  public void SetPosition(HoodPosition Position) {
    if (Position.equals(HoodPosition.CLOSE)) {
      hoodEncoder.setPosition(hoodClose);
    } else if (Position.equals(HoodPosition.MIDDLE)) {
      hoodEncoder.setPosition(hoodMiddle);
    } else if (Position.equals(HoodPosition.FAR)) {
      hoodEncoder.setPosition(hoodFar);
    }
  }

  public enum HoodPosition {
    CLOSE, MIDDLE, FAR;
  }
}