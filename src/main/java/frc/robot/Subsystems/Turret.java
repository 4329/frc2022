package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import frc.robot.Constants.*;
import frc.robot.Utilities.*;

//Turret class that extends the PIDSubsystem class which is used to allow command based programming
public class Turret extends PIDSubsystem {

  //Creates the TalonSRX for the feederMotor on the specified CAN ID
  private static TalonSRX m_turretMotor = new TalonSRX(TurretConstants.kTurretPort);
  //Creates the potentiometer for the absolute encoder for the turret and converts the voltage to radians and sets the tuned offset angle
  private static Potentiometer m_turretPotentiometer = new AnalogPotentiometer(TurretConstants.kTurretPotentiometerPort,
      2.0 * Math.PI, -2.80);

  private boolean searchClockwise = true; //creates boolean to indicate if the turret is searching clockwise for a target 
  private boolean trackTarget = false;    //creates boolean to indicate if the turret is in trackTarget mode

  public Turret() {
    //Super implementation that defines the PIDController for the PIDSubsystem with the PID values specified
    super(
        new PIDController(TurretConstants.kTurretPID[0], TurretConstants.kTurretPID[1], TurretConstants.kTurretPID[2]));
    getController().setTolerance(TurretConstants.kTurretTolerance);
    m_controller.setSetpoint(Math.PI);  //Defaults the setpoint to PI radians which is opposite of the turret deadzone
    m_turretMotor.setInverted(true);    //Motor direction is inverted so that positive values move the turret in the positive encoder direction

    //Enables voltage compensation on the TalonSRX to attempt to normalize output over wide range of bus voltage throughout a match
    m_turretMotor.configVoltageCompSaturation(GlobalConstants.kVoltCompensation);

    //Sets a max integrator range for the integral term of the PID, this term will most likely be removed once time can be made for creating a 
    //feedforward with a static gain value
    m_controller.setIntegratorRange(-TurretConstants.kTurretILimit, TurretConstants.kTurretILimit);
  }
  /**
   * Uses the output from the PIDController to set the motor%.
   * 
   * @param output is the PIDSubsystem's PIDController output motor%
   * @param setpoint is the PIDSubsystem's current desired setpoint
   */
  @Override
  public void useOutput(double output, double setpoint) {
    m_turretMotor.set(TalonSRXControlMode.PercentOutput, output);
  }
  /** 
   * 
   * @return the turret angle in radians
   */
  @Override
  public double getMeasurement() {
    return getPotentionmeter();
  }
  /** 
   * Function used to access the information on whether or not the Turret angle is at 
   * the desired setpoint. 
   * 
   * @return true if the turret is within the acceptable tolerance of the setpoint
   */
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
  /** 
   * This function is used to set the desired setpoint for the turret, it is fed the robot gyro reading which is
   * then modified by adding 3PI/2 and normalized to the unit circle angle. This is important because the turret
   * code is setup so that 0 radians (or 2PI) is in the deadzone so that a continous PID is not needed to cross over
   * the 0 to 2PI interface. It is important to note that driving the turret into the deadzone would damage the
   * wiring and therefore we do not want the Turret to take the shortest path to the setpoint as it would with
   * a continous PID
   * 
   * @param angle is the robot gyro angle in radians
   */
  public void setAngle(double angle) {

    //NOTE: due to the sensor oreintation on the turret, the sensor reads positive when the turret turns clockwise,
    //this is opposite of normal and clockwise being positive will be assumed for the following code
    //this will potentially be fixed later on but is low prioirty at this time

    //Transform gyro angle into terms of turret angle
    angle = MathUtils.toUnitCircAngle(3 * Math.PI / 2.0 + angle);

    //If trackTarget is set to true the turret will use the limelight to determine setpoint
    if (trackTarget) {
      Limelight.enable();
    } else {
      Limelight.disable();
    }

    //When the limelight does not have a valid solution, keep the turret facing the alliance wall and search back and
    //forth until a solution is found
    if (trackTarget && !Limelight.valid()) {
      if (searchClockwise && getPotentionmeter() > angle + Math.PI / 6.0) {
        searchClockwise = false;
      }
      if (!searchClockwise && getPotentionmeter() < angle - Math.PI / 6.0) {
        searchClockwise = true;
      }

      //when searching clockwise add 0.10 radians to the desired setpoint every isntance of the loop (20ms)
      //vice versa when searching counter clockwise
      if (searchClockwise) {
        angle = getPotentionmeter() + 0.10;
      } else {
        angle = getPotentionmeter() - 0.10;
      }
    } 
    
    //When the Limelight has a valid solution , use the limelight tx() angle and add it to the current turret postiion to 
    //determine the updated setpoint for the turret
    else if (trackTarget && Limelight.valid()) {
      angle = getPotentionmeter() + Limelight.tx();
    }
    //if the angle setpoint is lower than the minimum allowed position, set the setpoint to the minimum allowed position
    //and set the turret to search clockwise 
    if (angle < TurretConstants.kTurretLow) {
      searchClockwise = true;
      setSetpoint(TurretConstants.kTurretLow);
    } 
    //if the angle setpoint is hgiher than the maximum allowed position, set the setpoint to the maximum allowed position
    //and set the turret to search counter clockwise 
    else if (angle > TurretConstants.kTurretHigh) {
      searchClockwise = false;
      setSetpoint(TurretConstants.kTurretHigh);
    } 
    //when the setpoint is neither too high nor too low, set the desired setpoint to the last updated value of angle 
    else {
      setSetpoint(angle);
    }
  }

  /** 
   * Function to set the track target boolean to either true or false
   * 
   * @param track is true when Limelight tracking is desired
   */
  public void trackTarget(boolean track) {
    trackTarget = track;
  }

  /**
   * 
   * Obtains the potentiometer reading, this "if-else if-else" may be a remnant of buggy code and is proabbly unecessary now 
   */
   private double getPotentionmeter() {
    if (m_turretPotentiometer.get() > 2 * Math.PI) {
      return m_turretPotentiometer.get() - 2 * Math.PI;
    } else if (m_turretPotentiometer.get() < 0.0) {
      return m_turretPotentiometer.get() + 2 * Math.PI;
    } else {
      return m_turretPotentiometer.get();
    }
  }


  /**
   * Because the same PIDController is used for the turret at all times, a simple atSetpoint() call is not good enough to know it is
   * okay to shoot. Instead this fucntion is defined for when there is a valid solution and the limelight value is within the allowable
   * tolerance
   */
  public boolean visionAligned() {
    if (Limelight.valid() && Math.abs(Limelight.tx()) < VisionConstants.kTrackTolerance) {
      return true;
    } else {
      return false;
    }
  }
}
