package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import frc.robot.Constants.*;

public class Turret extends PIDSubsystem {
 
    private static TalonSRX m_turretMotor = new TalonSRX(TurretConstants.kTurretPort);
    private static Potentiometer m_turretPotentiometer = new AnalogPotentiometer(TurretConstants.kTurretPotentiometerPort, 2.0*Math.PI, -2.80);

    private boolean searchClockwise = true;
    private boolean trackTarget = false;

    public Turret() {
        super(
            new PIDController(
                TurretConstants.kTurretPID[0], 
                TurretConstants.kTurretPID[1], 
                TurretConstants.kTurretPID[2]));
        getController().setTolerance(TurretConstants.kTurretTolerance);
        m_controller.setSetpoint(Math.PI);
        m_turretMotor.setInverted(true);
        m_controller.setIntegratorRange(-TurretConstants.kTurretILimit, TurretConstants.kTurretILimit);
      }

      @Override
      public void useOutput(double output, double setpoint) {
        SmartDashboard.putNumber("Turret Output", output);
        m_turretMotor.set(TalonSRXControlMode.PercentOutput,output);
      }

      @Override
      public double getMeasurement() {
          SmartDashboard.putNumber("Turret Location", getPotentionmeter());
        return getPotentionmeter();
      }

      public boolean atSetpoint() {
        return m_controller.atSetpoint();
      }

      public void setAngle(double angle){
        SmartDashboard.putNumber("Turret Error", m_controller.getPositionError());
            if(trackTarget)
            {
              Limelight.enable();
            }
            else
            {
              Limelight.disable();
            }
        
            if(trackTarget && !Limelight.valid())
            {
              if(searchClockwise && getPotentionmeter() > angle+Math.PI/6.0)
              {
                searchClockwise = false;
              }
              if(!searchClockwise && getPotentionmeter() < angle-Math.PI/6.0)
              {
                searchClockwise = true;
              }
              if(searchClockwise)
              {
                angle = getPotentionmeter()+0.10;
              }
              else
              {
                angle = getPotentionmeter()-0.10;
              }

            }
            else if(trackTarget && Limelight.valid()){
              angle = getPotentionmeter()+Limelight.tx();
            }
            if(angle < TurretConstants.kTurretLow){
                setSetpoint(TurretConstants.kTurretLow);
            }
            else if(angle > TurretConstants.kTurretHigh) {
                setSetpoint(TurretConstants.kTurretHigh);
            }
            else{
                setSetpoint(angle);
            }
            SmartDashboard.putNumber("Turret Desired Angle", angle);
    }

      public void trackTarget(boolean track){
        trackTarget = track;      
    }
      public double getDistance(){
        if(Limelight.valid())
        {
          return Limelight.getDistance();
        }
        else
        {
          return Limelight.getDistance();
        }
      }

      private double getPotentionmeter(){
          if(m_turretPotentiometer.get() > 2*Math.PI){
              return m_turretPotentiometer.get() - 2*Math.PI;
          }
          else if(m_turretPotentiometer.get() < 0.0){
              return m_turretPotentiometer.get() + 2*Math.PI;
          }
          else{
              return m_turretPotentiometer.get();
          }
      }
      
      public boolean visionAligned(){
        if(Limelight.valid() && Math.abs(Limelight.tx())<0.0140){
          return true;
        }
        else{
          return false;
        }
        }
      public boolean visionNotAligned(){
        return !visionAligned();
      }
      
}
