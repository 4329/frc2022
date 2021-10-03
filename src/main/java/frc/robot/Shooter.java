package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;


import frc.robot.Constants.*;

public class Shooter extends PIDSubsystem {
    
    private final CANSparkMax m_shooterMotor1 = new CANSparkMax(ShooterConstants.kMotorPorts[0], MotorType.kBrushless);
    private final CANSparkMax m_shooterMotor2 = new CANSparkMax(ShooterConstants.kMotorPorts[1], MotorType.kBrushless);
    private final CANEncoder m_shooterEncoder1 = m_shooterMotor1.getEncoder();
    private final CANEncoder m_shooterEncoder2 = m_shooterMotor2.getEncoder();

    private static DoubleSolenoid flap = new DoubleSolenoid(0, 4);
    private static boolean currentlyClose = true;

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

        m_shooterMotor1.burnFlash();
        m_shooterMotor2.burnFlash();
        
      }

      @Override
      public void useOutput(double output, double setpoint) {
        m_shooterMotor1.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
        m_shooterMotor2.follow(m_shooterMotor1, true);
      }
    
      @Override
      public double getMeasurement() {
        return m_shooterEncoder1.getVelocity();
      }

      public boolean atSetpoint() {
        return m_controller.atSetpoint();
      }

      public void setDistance(double distance) {
          double rpmCommand = 4000;
        if (distance > 275.0 && currentlyClose) {
            currentlyClose = false;
        } else if (distance < 220.0 && !currentlyClose) {
            currentlyClose = true;
        }
        if (currentlyClose) {
            rpmCommand = -0.0003857 * Math.pow(distance, 3) + 0.3357896 * Math.pow(distance, 2) - 94.3357106 * distance + 11587.3458;
            setFlap(false);
        } else {
            rpmCommand = 4000.0;
            setFlap(true);
        }
          m_controller.setSetpoint(rpmCommand);
      }

      public static void setFlap(boolean down) {
        if (down) {
            flap.set(DoubleSolenoid.Value.kForward);
        } else {
            flap.set(DoubleSolenoid.Value.kReverse);
        }
    }

    
      public void runFeeder() {
        m_feederMotor.set(TalonSRXControlMode.PercentOutput, ShooterConstants.kFeederSpeed);
      }
    
      public void stopFeeder() {
        m_feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
      }
}
