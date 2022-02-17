package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import frc.robot.Constants.*;
import frc.robot.Utilities.*;

  /**
   * Implements a Turret PIDSubsystem for the robot
   */
  public class Turret extends PIDSubsystem {

  //Creates the TalonSRX for the feederMotor on the specified CAN ID
  private final TalonSRX m_turretMotor = new TalonSRX(TurretConstants.kTurretPort);
  private final AS5600EncoderPwm turretEncoder = new AS5600EncoderPwm(m_turretMotor.getSensorCollection());
  //Creates the potentiometer for the absolute encoder for the turret and converts the voltage to radians and sets the tuned offset angle
  //private static AnalogPotentiometer m_turretPotentiometer = new AnalogPotentiometer(TurretConstants.kTurretPotentiometerPort,
  //    2.0 * Math.PI, -2.80);
  private boolean searchClockwise = true; //creates boolean to indicate if the turret is searching clockwise for a target 
  private boolean trackTarget = false;    //creates boolean to indicate if the turret is in trackTarget mode
  private boolean visionSolution = false;
  public double turretEncoderValues = turretEncoder.getPwmPosition();
  public double maxTurretEncoderValue = turretEncoder.getPwmPosition();
  public double minTurretEncoderValue = turretEncoder.getPwmPosition();
  public double turretEncoderRadians =  (turretEncoder.getPwmPosition() / 4029 )* 2 * Math.PI;
  public double limeLightTxValReplacement = 20; //-27 to 27, in degrees

  //max 3964      :one tick is 0.0893521966 degrees
  //min -127      :total is 4029
  //forward -125  :one tick is 0.000496401092 pi radians
  //back  1913    :one radian is 

  NetworkTableEntry turretEncoderPulses = Shuffleboard.getTab("Swerve Alignment").add("Turret Location in Pulses", turretEncoderValues).withPosition(8,0).getEntry();
  
  public void displayTurretEncoderPulses() {
    turretEncoderValues = turretEncoder.getPwmPosition();
    System.out.println("encoder pulses called " + turretEncoder.getPwmPosition());
    turretEncoderPulses.setDouble(turretEncoder.getPwmPosition());
}

  /**
   * Creates a Turret PIDSubsystem and sets the appropirate values for the motor controllers, analog encoder, and PIDController.
   * Because there will only ever be 1 turret on the robot the appropriate values will be pulled in from the 
   * Constants classses. If multiple turrets are required (unlikely) then the class contructor must be modified to read in
   * the appropriate values for each motor controller and absolute encoder.
   * 
   */
  public Turret() {
    //Super implementation that defines the PIDController for the PIDSubsystem with the PID values specified
    super(
        new PIDController(TurretConstants.kTurretPID[0], TurretConstants.kTurretPID[1], TurretConstants.kTurretPID[2]));
    getController().setTolerance(TurretConstants.kTurretTolerance);
    m_controller.setSetpoint(Math.PI);  //Defaults the setpoint to PI radians which is opposite of the turret deadzone
    m_turretMotor.setInverted(true);    //Motor direction is inverted so that positive values move the turret in the positive encoder direction

    //Enables voltage compensation on the TalonSRX to attempt to normalize output over wide range of bus voltage throughout a match
    m_turretMotor.configVoltageCompSaturation(GlobalConstants.kVoltCompensation);
  }
  /**
   * Uses the output from the PIDController to set the motor%.
   * 
   * @param output is the PIDSubsystem's PIDController output motor%
   * @param setpoint is the PIDSubsystem's current desired setpoint
   */
  @Override
  public void useOutput(double output, double setpoint) { //LOOK AT DIS ASAP SOON AZ POSSIBLE
    final double staticGain = TurretConstants.kStaticGain*Math.signum(output);
    m_turretMotor.set(TalonSRXControlMode.PercentOutput, output+staticGain);
    System.out.println("mie lief iz pane");
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

    visionSolution = /*Limelight.valid() && */(Math.abs(getPotentionmeter()-angle) < Math.PI/6.0);

    //When the limelight does not have a valid solution, keep the turret facing the alliance wall and search back and
    //forth until a solution is found
    if (trackTarget && !visionSolution) {
      if (searchClockwise && getPotentionmeter() > angle + Math.PI / 6.0) {
        searchClockwise = false;
      }
      if (!searchClockwise && getPotentionmeter() < angle - Math.PI / 6.0) {
        searchClockwise = true;
      }

      //when searching clockwise add 0.10 radians to the desired setpoint every isntance of the loop (20ms)
      //vice versa when searching counter clockwise
      if (searchClockwise) {
        angle = getPotentionmeter() + 0.20;
      } else {
        angle = getPotentionmeter() - 0.20;
      }
    } 
    
    //When the Limelight has a valid solution , use the limelight tx() angle and add it to the current turret postiion to 
    //determine the updated setpoint for the turret
    else if (trackTarget && visionSolution) {
      angle = getPotentionmeter() + limeLightTxValReplacement; //Limelight.tx()
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
    if (turretEncoderRadians > 2 * Math.PI) {
      return turretEncoderRadians - 2 * Math.PI;
    } else if (turretEncoderRadians < 0.0) {
      return turretEncoderRadians + 2 * Math.PI;
    } else {
      return turretEncoderRadians;
    }
  }


  /**
   * Because the same PIDController is used for the turret at all times, a simple atSetpoint() call is not good enough to know it is
   * okay to shoot. Instead this fucntion is defined for when there is a valid solution and the limelight value is within the allowable
   * tolerance
   */
  public boolean visionAligned() {
    if (/*Limelight.valid() &&*/ Math.abs(limeLightTxValReplacement) < VisionConstants.kTrackTolerance) {
      return true;
    } else {
      return false;
    }
  }
}//Limelight.tx()