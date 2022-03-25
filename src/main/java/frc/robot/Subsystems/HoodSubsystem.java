//Written by Ben Durbin, 2022

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.Configrun;

public class HoodSubsystem extends SubsystemBase {
  private PIDController hoodPID;
  private CANSparkMax hoodwheel;
  private RelativeEncoder hoodEncoder;
  private int output;
  private NetworkTableEntry sparkPosition;
  private NetworkTableEntry hoodOverrideIdleMode;
  private NetworkTableEntry overrideSetpointEntry;
  private NetworkTableEntry preHood;
  private double hoodOpen;
  private double hoodHalf;
  private double hoodClosed;
  private double hoodNeutral;
  private static final int MAX_RANGE = 33;
  private double setpoint;
  private double overrideSetpoint;




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

    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      sparkPosition = Shuffleboard.getTab("Hood Data").add("Position", hoodEncoder.getPosition())
          .withWidget(BuiltInWidgets.kTextView)
          .withPosition(0, 1).withSize(1, 1).getEntry();

      hoodOverrideIdleMode = Shuffleboard.getTab("Hood Data").add("Override Idle Mode", true)
          .withWidget(BuiltInWidgets.kToggleButton)
          .withSize(2, 1).getEntry();
    
      overrideSetpointEntry = Shuffleboard.getTab("Hood Data").add("Hood Setpoint", 3).withWidget(BuiltInWidgets.kTextView).getEntry();

      preHood = Shuffleboard.getTab("Hood Data").add("preHood", 3).getEntry();

      Shuffleboard.getTab("Hood Data").add("Read Me", "If functional: Made by Ben Durbin Else: Made by Mr. Emerick")
          .withWidget(BuiltInWidgets.kTextView).withPosition(5, 0).withSize(3, 1);
    }

    
  }

  /**
   * This method actually runs the hood, but can only be influenced by setPosition()
   * and setEncoderPosition()
   * 
   * @param shooter
   */
  public void HoodPeriodic(Shooter shooter) {
    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      preHood.setDouble(output);
    }
    double hoodposition = hoodEncoder.getPosition();
    //System.out.println("Hood Position" + hoodposition);
    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      sparkPosition.setDouble(hoodposition);
    }

   

   if (shooter.manualOverride.getBoolean(true)) {

        setpoint = overrideSetpoint;
   }
   

    double output = hoodPID.calculate(hoodposition, setpoint);

    output = output / MAX_RANGE;

    if (Math.abs(hoodposition) > MAX_RANGE) {
      output = 0;
    }
    hoodwheel.set(output);
  }

  // System.out.println(hoodEncoder.getPosition());

  /**
   * @return whether or not the hood is within tolerance
   */
  public boolean hoodSet() {
    return hoodPID.atSetpoint();
  }

  /**
   * stops the hood
   */
  public void stop() {
    hoodwheel.set(0);
  }

  /**
   * Sets the hood position with a HoodPosition
   * 
   * @param Position
   */
  public void setPosition(HoodPosition position) {  
    
      if (position.equals(HoodPosition.OPEN)) {
        setpoint = hoodOpen;
      } else if (position.equals(HoodPosition.HALF)) {
        setpoint = hoodHalf;
      } else if (position.equals(HoodPosition.CLOSED)) {
        setpoint = hoodClosed;
      } else if (position.equals(HoodPosition.NEUTRAL)) {
        setpoint = hoodNeutral;
      }
  }

  /**
   * Sets the hood position with a double
   * 
   * @param position
   */
  public void setEncoderPosition(double position) {
    if (position < 3) {
      setpoint = 3;
    } else if (position > 33) {
      setpoint = 33;
    } else {
       setpoint = position;
    }
  }

 
  

  public enum HoodPosition {

    OPEN, HALF, CLOSED, NEUTRAL;
  }

  /**
   * Code in here is called upon test mode's being enabled
   */
  public void hoodTestMode() {
    hoodwheel.setIdleMode(IdleMode.kCoast);
    double hoodposition = hoodEncoder.getPosition();
    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      sparkPosition.setDouble(hoodposition);
    }
  }

  /**
   * Allows one to set hood position and idle mode directly through shuffleboard if 
   * manual override is enabled
   * 
   * @param shooter
   */
  public void hoodOverride(Shooter shooter) {
    if (Configrun.get(false, "extraShuffleBoardToggle")) {
      sparkPosition.setDouble(hoodEncoder.getPosition());
    }
      HoodPeriodic(shooter);
      if (shooter.manualOverride.getBoolean(true)) {

        if (hoodOverrideIdleMode.getBoolean(true)) {
          
          setEncoderPosition(overrideSetpointEntry.getDouble(3));
          hoodwheel.setIdleMode(IdleMode.kBrake);
        }
      else {

        hoodwheel.setIdleMode(IdleMode.kCoast);

        }
      }

    }
}