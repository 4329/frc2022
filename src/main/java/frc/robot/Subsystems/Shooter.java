package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;

public class Shooter extends PIDSubsystem {
    
    private final CANSparkMax m_shooterMotor1 = new CANSparkMax(ShooterConstants.kMotorPorts[0], MotorType.kBrushless);
    private final CANSparkMax m_shooterMotor2 = new CANSparkMax(ShooterConstants.kMotorPorts[1], MotorType.kBrushless);
    private final CANEncoder m_shooterEncoder1 = m_shooterMotor1.getEncoder();

    private static DoubleSolenoid flap = new DoubleSolenoid(0, 4);
    private static boolean currentlyClose = true;
    private static boolean loading = false;

    private final DigitalInput limit1 = new DigitalInput(ShooterConstants.kLimitSwitchPorts[0]);
    private final DigitalInput limit2 = new DigitalInput(ShooterConstants.kLimitSwitchPorts[1]);

    private final TalonSRX m_feederMotor = new TalonSRX(ShooterConstants.kFeederPort);

    private final SimpleMotorFeedforward m_shooterFeedforward =
    new SimpleMotorFeedforward(
        ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

    public Shooter() {
        super(
            new PIDController(
                ShooterConstants.kShooterPID[0], 
                ShooterConstants.kShooterPID[1], 
                ShooterConstants.kShooterPID[2]));
        getController().setTolerance(ShooterConstants.kShotHysteresis);
        m_shooterMotor1.setSmartCurrentLimit(60, 40);
        m_shooterMotor2.setSmartCurrentLimit(60, 40);
        m_shooterMotor1.setInverted(true);
        m_shooterMotor1.enableVoltageCompensation(12.6);
        m_shooterMotor1.burnFlash();
        m_shooterMotor2.burnFlash();   
        
        setSetpoint(3000.0);
      }

      @Override
      public void useOutput(double output, double setpoint) {
          SmartDashboard.putNumber("FeedFor", m_shooterFeedforward.calculate(setpoint));
          SmartDashboard.putNumber("OutputShooter", output);

        //m_shooterMotor1.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
        m_shooterMotor1.set(output/12.0 + m_shooterFeedforward.calculate(setpoint)/12.0);
        m_shooterMotor2.follow(m_shooterMotor1, true);
      }
    
      @Override
      public double getMeasurement() {
          SmartDashboard.putNumber("Shooter Error", m_controller.getPositionError());
          SmartDashboard.putNumber("Encoder Velocity", m_shooterEncoder1.getVelocity());
        return m_shooterEncoder1.getVelocity();
      }

      public boolean atSetpoint() {
        return m_controller.atSetpoint();
      }

      public void setRPM(double distance) {
        double RPMcommand = 3000.0;
        SmartDashboard.putNumber("Distance", distance);
        if (distance > 240.0 && currentlyClose) {
            currentlyClose = false;
        } else if (distance < 220.0 && !currentlyClose) {
            currentlyClose = true;
        }

        if (currentlyClose) {
            RPMcommand = 0.00021811*Math.pow(distance, 4) - 0.14587931*Math.pow(distance, 3) + 36.33601566*Math.pow(distance, 2) -4001.14*distance + 167946.35 - 225.0;
            if(distance >210.0)
            {
                RPMcommand = 3075+(distance-210)*13.333;
            }
            else if(distance > 225)
            {
                RPMcommand = 3175;
            }
            setFlap(false);
        } else {
            RPMcommand = 4150.0;
            setFlap(true);
        }
        setSetpoint(RPMcommand);
      }

      private void setFlap(boolean down) {
        if (down) {
            flap.set(DoubleSolenoid.Value.kForward);
        } else {
            flap.set(DoubleSolenoid.Value.kReverse);
        }
    }

    
      public void runFeeder() {
        if(isEnabled() && atSetpoint())
        {
        m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
        }
    }
    
      public void stopFeeder() {
        m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
      }

      public void feedThroat() {
        if(!isEnabled() && (!limit1.get() && !limit2.get()))
        {
          m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kThroatSpeed);
          loading = true;
        }
        else if(!isEnabled() && loading && !limit2.get()){
          m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kThroatSpeed);
        }
        else if(!isEnabled() && limit2.get()){
          loading = false;
          stopFeeder();
        }
      }
}
