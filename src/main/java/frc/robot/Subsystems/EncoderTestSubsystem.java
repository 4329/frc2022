package frc.robot.Subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class EncoderTestSubsystem extends SubsystemBase{

    //TalonSRX testMotor = new TalonSRX(40);
    private final TalonSRX testMotor = new TalonSRX(40);
    private final AS5600EncoderPwm testEncoder = new AS5600EncoderPwm(testMotor.getSensorCollection());
    public double testEncoderValues = testEncoder.getPwmPosition();
    double turretSpeed = 0.2;
    NetworkTableEntry encoderPulses = Shuffleboard.getTab("RobotData").add("Encoder Pulses", testEncoderValues).getEntry();
    NetworkTableEntry encoderPulsesTest = Shuffleboard.getTab("Swerve Alignment").add("Turret Location in Pulses", testEncoderValues).withPosition(8,0).getEntry();


    public void motorRight() {
        testMotor.set(ControlMode.PercentOutput, -turretSpeed);
    }

    public void motorLeft() {
        testMotor.set(ControlMode.PercentOutput, turretSpeed);
    }

    public void stopMotor() {
        testMotor.set(ControlMode.PercentOutput, 0);
    }

    public void getEncoderPulses() {
        testEncoderValues = testEncoder.getPwmPosition();
        System.out.println("encoder pulses called " + testEncoder.getPwmPosition());
        encoderPulses.setDouble(testEncoder.getPwmPosition());
        encoderPulsesTest.setDouble(testEncoder.getPwmPosition());
    }

    public boolean isFinished() {
        return false;
    }
}
