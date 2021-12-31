package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Constants.*;
import java.awt.geom.Point2D;
import frc.robot.Utilities.LinearInterpolationTable;

  /**
   * Implements a Shooter PIDSubsystem for the robot
   */
  public class Shooter extends PIDSubsystem {

  //Creates the SparkMAXs for each shooter motor at the defined CANID
  private final CANSparkMax m_shooterMotor1 = new CANSparkMax(ShooterConstants.kMotorPorts[0], MotorType.kBrushless);
  private final CANSparkMax m_shooterMotor2 = new CANSparkMax(ShooterConstants.kMotorPorts[1], MotorType.kBrushless);
  //Only one of the motor encoders is needed to feed the PIDcontroller
  private final CANEncoder m_shooterEncoder1 = m_shooterMotor1.getEncoder();
  //Create double solenoid for the shooter "flap"
  private static DoubleSolenoid flap = new DoubleSolenoid(ShooterConstants.kFlapSolenoids[0], 
      ShooterConstants.kFlapSolenoids[1]);
  
  private static boolean currentlyClose = true; //Create a boolean to track if the robot is currently in the close flap position
  private static boolean loading = false;       //Create boolean to track if the robot is currently loading a ball into the throat

  private final DigitalInput limitFeed = new DigitalInput(ShooterConstants.kLimitSwitchPorts[0]);  //Create normally open limit switch object for loading the throat
  private final DigitalInput limitThroat = new DigitalInput(ShooterConstants.kLimitSwitchPorts[1]);  //Create normally closed limit switch object for loading the throat

  private final TalonSRX m_feederMotor = new TalonSRX(ShooterConstants.kFeederPort);  //Creates the TalonSRX for the feederMotor on the specified CAN ID

  private final Timer m_enableTimer = new Timer();

  private Point2D[] interpolationTable = 
    new Point2D.Double[]{
      new Point2D.Double(100,4525),
      new Point2D.Double(125,3700),
      new Point2D.Double(150,3175),
      new Point2D.Double(175,3025),
      new Point2D.Double(200,2900),
      new Point2D.Double(225,3150),
      new Point2D.Double(250,3250),
    };  
  private LinearInterpolationTable m_RPMTable = new LinearInterpolationTable(interpolationTable);

  //Creates a SimpleMotorFeedForward with the specified feedforward gain
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.kStaticGain,
      ShooterConstants.kShooterFF);

  /**
   * Creates a Shooter PIDSubsystem and sets the appropirate values for the motor controllers and PIDController.
   * Because there will only ever be 1 shooter on the robot the appropriate values will be pulled in from the 
   * Constants classses. If multiple shooters are required (unlikely) then the class contructor must be modified to read in
   * the appropriate values for each motor controller.
   * 
   */
    public Shooter() {
    //Super implementation that defines the PIDController for the PIDSubsystem with the PID values specified
    super(new PIDController(ShooterConstants.kPID[0], ShooterConstants.kPID[1], ShooterConstants.kPID[2]));

    //Sets the PIDController tolerance to the specified value so that the atSetpoint() function returns truw onyl when the error is below this value
    getController().setTolerance(ShooterConstants.kShotRPMTolerance); 

    m_shooterMotor1.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);  //Sets the current limit for the SparkMAX Controller
    m_shooterMotor2.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);  //Sets the current limit for the SparkMAX Controller
    m_shooterMotor1.setInverted(true);                                            //Inverts the direction of motor 1 so that positive shooter RPMs shoot the ball
    m_shooterMotor1.enableVoltageCompensation(GlobalConstants.kVoltCompensation); //Enable voltage compensation so gains scale with bus voltage
    m_shooterMotor2.enableVoltageCompensation(GlobalConstants.kVoltCompensation); //Enable voltage compensation so gains scale with bus voltage

    m_shooterMotor1.burnFlash();                                                  //Write parameters to the sparkMAX so it is certain the values are set
    m_shooterMotor2.burnFlash();                                                  //Write parameters to the sparkMAX so it is certain the values are set

    setSetpoint(3000.0);                                                          //It is important to set a default value here that is not 0RPMs, this ensures the
                                                                                  //atSetpoint() function will not return true when the shooter is not spun up yet
                                                                                  //and could potentially jam and stall the shooter with a ball prematurely                          


  }
  /**
   * Uses the output from the PIDController combined with the feedforward calculation to 
   * set the motor%. The 2nd motor is set as an inverted follower to the 1st motor as it will
   * spin the opposite direction. 
   * 
   * @param output is the PIDSubsystem's PIDController output motor%
   * @param setpoint is the PIDSubsystem's current desired setpoint
   */
  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterMotor1.set(output + m_shooterFeedforward.calculate(setpoint));
    m_shooterMotor2.follow(m_shooterMotor1, true);
  }
  /** 
   * 
   * @return the shooter motor velocity in RPMs
   */
  @Override
  public double getMeasurement() {
    return m_shooterEncoder1.getVelocity();
  }
  @Override
  public void enable() {
    m_enabled = true;
    m_controller.reset();
    m_enableTimer.reset();
    m_enableTimer.start();
  }

  /** 
   * 
   * Function used to access the information on whether or not the Shooter RPM is at 
   * the desired setpoint. 
   * 
   * @return true if the shooter velocity is within the acceptable tolerance of the setpoint
   */
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  /** 
   * Function used to set the desired setpoint in RPMs for the shooter motors. The functionality
   * of the shooter flap is also contained within this function.
   * 
   * @param distance in inches from the shooter to the target center in inches
   */
  public void setRPM(double distance) {
    double RPMcommand = 3000.0; //Create a double to store the RPM command value and set it to a reasonable default value

    //The follwoing if statements create a typical hysteresis for the currentlyClose var so that the mechanism can not be in a position where
    //sensor noise will cause continual actuation back and forth
    if (distance > ShooterConstants.kFlapDownDist && currentlyClose) {
      currentlyClose = false;
    } else if (distance < ShooterConstants.kFlapUpDist && !currentlyClose) {
      currentlyClose = true;
    }

    if (currentlyClose) {
      RPMcommand = m_RPMTable.getOutput(distance);
    } else {
      //Cosntant RPM for flap down shooting mode
      RPMcommand = 4350.0;
      //When not currently close the flap solenoid should be extended position
      setFlap(true);
    }
    //set the PIDController setpoint to the desired RPM command
    setSetpoint(RPMcommand);
  }

  /** 
   * Function to set the shooter to a persistent RPM with the flap up
   * 
   * @param rpm desired for persistent rpm
   */  
  public void setPersistentRPM(double rpm){
    setSetpoint(rpm);
    setFlap(false);
  }

  /** 
   * Function to set the flap down or up
   * 
   * @param down will lower the flap when true
   */  
  private void setFlap(boolean down) {
    if (down) {
      flap.set(DoubleSolenoid.Value.kForward);
    } else {
      flap.set(DoubleSolenoid.Value.kReverse);
    }
  }
  /** 
   * Runs the feeder motor in reverse. Is very useful when needed to unjam stuck balls in the hopper.
   * Will only run when the shooter is currently active.
   */  
  public void reverseFeeder() {
    if (isEnabled()) {
      m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kFeederReverseSpeed);
    }
  }
  /** 
   * Runs the feeder motor to feed balls into the shooter.
   * Will only run when the shooter is active and at the desired setpoint.
   */  
  public void runFeeder() {
    if (isEnabled() && atSetpoint()) {
      m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
    }
  }
  /** 
   * Stops the feeder motor by setting the desired power to 0%. This is very important to call anytime a command is interupted
   * so the shooter is not stuck with the feeder motor running when not intended
   */  
  public void stopFeeder() {
    loading = false;
    m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /** 
   * This function allows the feeder to preload the throat with 1 ball when avialable by tracking the position of the balls
   * in the hopper and throat by the use of limit switches. This only runs when the shooter is not enabled. Upon enabling of the 
   * shooter, the feeder motor is automatically stopped so that it can not get stuck in a feed position when not desired.
   */  
  public void feedThroat() {
    //DIO ports read "true" when a connection is open. This means a normally open switch reads true until 
    //pressed and vice versa for a normally closed switch. For the following the logic reads: if the shooter is not enabled
    //and the NO limitFeed is pressed (which reads in as false) and the NC limitThroat switch is not pressed (which reads in as false) 
    //then set the loading boolean to true and run the feeder at the specified speed
    if ((!isEnabled() && !limitFeed.get() && !limitThroat.get())) {
      m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kThroatSpeed);
      loading = true;
    } 
    //The follwing logic reads: If the shooter is not enabled and it is currently loading and the NC limit switch is not pressed 
    //(which reads in as false) continue to set the feed motor at the specificed speed
    else if (!isEnabled() && loading && !limitThroat.get()) {
      m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kThroatSpeed);
    } 
    //Finally the following logic reads: If the shooter is not enabled and the NC limit switch is pressed (which reads in as true)
    //then run the stopFeeder() function which sets the loading boolean to false and stops the feeder motor
    else if (!isEnabled() && limitThroat.get()) {
      stopFeeder();
    }
  }

  public double getTotalCurrent() {
    return m_shooterMotor1.getOutputCurrent() + m_shooterMotor2.getOutputCurrent();
  }

  public double timeSinceEnabled() {
    return m_enableTimer.get();
  }

}
