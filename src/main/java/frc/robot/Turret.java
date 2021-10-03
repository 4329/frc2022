package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import frc.robot.Constants.*;

public class Turret extends PIDSubsystem {
 
    private static TalonSRX m_turretMotor = new TalonSRX(TurretConstants.kTurretPort);
    private static Potentiometer m_turretPotentiometer = new AnalogPotentiometer(TurretConstants.kTurretPotentiometerPort, 2.0*Math.PI, 3.1835);

    public Turret() {
        super(
            new PIDController(
                TurretConstants.kTurretPID[0], 
                TurretConstants.kTurretPID[1], 
                TurretConstants.kTurretPID[2]));
        getController().setTolerance(TurretConstants.kTurretTolerance);        
        m_controller.setSetpoint(Math.PI);
      }

      @Override
      public void useOutput(double output, double setpoint) {
        m_turretMotor.set(TalonSRXControlMode.PercentOutput,output);
      }

      @Override
      public double getMeasurement() {
        return m_turretPotentiometer.get();
      }

      public boolean atSetpoint() {
        return m_controller.atSetpoint();
      }

      public void setAngle(double angle){
        if(angle < TurretConstants.kTurretLow){
            setSetpoint(TurretConstants.kTurretLow);
        }
        else if(angle > TurretConstants.kTurretHigh) {
            setSetpoint(TurretConstants.kTurretHigh);
        }
        else{
            setSetpoint(angle);
        }
      }

      public void trackTarget(){
          setSetpoint(Math.PI);
      }
      public double getDistance(){
        return 240;
      }

}
