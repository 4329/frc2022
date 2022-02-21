package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Turret extends PIDSubsystem{
    private final static int MAX_TICKS = 4128;
    private final static int MAX_LEFT = 1000;
    private final static int MAX_RIGHT = 3000;
    private TalonSRX turretmotor;
    private AS5600EncoderPwm turretencoder;
    private NetworkTableEntry encodershufflboard;
    private NetworkTableEntry rawEncodershufflboard;
    private NetworkTableEntry turretSetpoint;

    public Turret () {
        super (new PIDController(6, 0, 0));
        turretmotor = new TalonSRX(44);
        turretencoder = new AS5600EncoderPwm(turretmotor.getSensorCollection());
        m_controller.setTolerance(10);
        turretmotor.setNeutralMode(NeutralMode.Brake);
        turretmotor.setInverted(InvertType.InvertMotorOutput);

        encodershufflboard = Shuffleboard.getTab("turret") .add("current turret encoder value" , turretencoder.getPwmPosition()).withWidget(BuiltInWidgets.kTextView).getEntry();
        rawEncodershufflboard = Shuffleboard.getTab("turret") .add("raw current turret encoder value" , turretencoder.getRawPosition()).withWidget(BuiltInWidgets.kTextView).getEntry();
        turretSetpoint = Shuffleboard.getTab("turret") .add("Turret Setpoint" , turretencoder.getPwmPosition()).withWidget(BuiltInWidgets.kTextView).getEntry();


    }
    public void showturretencoder() {
        encodershufflboard.setDouble(turretencoder.getPwmPosition());
        rawEncodershufflboard.setDouble(turretencoder.getRawPosition());
      //m_controller.setSetpoint(turretSetpoint.getDouble(0));
    }

    @Override
    public void periodic() {
        super.periodic();
        showturretencoder();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        System.out.println("output is " + output + " and setpoint is: " + setpoint);

        // see if this works ok... 
        double percentage = output / MAX_TICKS;
        turretmotor.set(TalonSRXControlMode.PercentOutput, percentage);
    }


    @Override
    protected double getMeasurement() {
        int position = turretencoder.getPwmPosition();
        System.out.println("getting measurement for pid..., postiion: " + position);
        return position;
    }


    public void setPositonFromShuffleBoard() {
        System.out.println("getting position from shuffleboard");
        setSetpoint(turretSetpoint.getDouble(0));
        enable();
       // setSetpoint(4000);
    }

    public boolean atSetpoint() {
        return m_controller.atSetpoint();
    } 
}