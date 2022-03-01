package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

public class TurretSubsystem extends SubsystemBase{

    private static final double LIMELIGHT_RANGE = 30;
    private static final double TURRET_RANGE = 4029;

    private TalonSRX turret;
    private volatile int lastValue = Integer.MIN_VALUE;

    double staticFeedforward = 0;
    // default value for the limelight mode
    int defaultvalue = 1;
    PIDController limeLightPid;
    PIDController turretPid;
    double h1In = Configrun.get(36, "h1In");
    // height of the limelight off of the ground "36 inches" this year
    double h2In = Configrun.get(104, "h2In");
    // height of the target "8ft 8inches" aka 104 inches this year
    double a1Degree = Configrun.get(0.0, "a1Degree");
    // angle of the front of the limelight in relation to level
    double limeLightDistance;
    int limeLightTolerance = 1;
    int turretTolerance = 1;
    double taTolerance = 0.3;

    public double currentDistance = 120;

    private NetworkTableEntry checkTXDisplay;
    private NetworkTableEntry checkTYDisplay;
    private NetworkTableEntry checkTADisplay;
    private NetworkTableEntry checkTVDisplay;
    private NetworkTableEntry getDistanceFromTargetDisplay;
    private NetworkTableEntry turretPos;
    private NetworkTableEntry turretRotationMin;
    private NetworkTableEntry turretRotationMax;
    NetworkTableEntry targetStatus;

    public TurretSubsystem() {
        turret = new TalonSRX (Configrun.get(41, "turretID"));
        limeLightPid = new PIDController(1, 0, 0);
        limeLightPid.setTolerance(limeLightTolerance);
        turretPid = new PIDController(1, 0, 0);
        turretPid.setTolerance(turretTolerance);
        targetStatus = Shuffleboard.getTab("RobotData").add("Target Acquired", false).getEntry();
        checkTXDisplay = Shuffleboard.getTab("Limlight").add("Tx", 0).withPosition(3, 1).getEntry();
        checkTYDisplay = Shuffleboard.getTab("Limlight").add("TY", 0).withPosition(3, 0).getEntry();
        checkTADisplay = Shuffleboard.getTab("Limlight").add("TA", 0).withPosition(4, 0).getEntry();
        checkTVDisplay = Shuffleboard.getTab("Limlight").add("TV", 0).withPosition(4, 1).getEntry();
        getDistanceFromTargetDisplay = Shuffleboard.getTab("Limlight").add("Distance", 0).withPosition(5, 0).getEntry();
        turretPos = Shuffleboard.getTab("Limlight").add("Turret Position", getPwmPosition()).withPosition(3, 2).getEntry();
        turretRotationMin = Shuffleboard.getTab("Limlight").add("Turret Minimum", getPwmPosition() - 307).withPosition(3, 3).getEntry();
        turretRotationMax = Shuffleboard.getTab("Limlight").add("Turret Maximum", getPwmPosition() + 307).withPosition(3, 4).getEntry();
    }

    public void putValuesToShuffleboard() {
        checkTXDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
        checkTYDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
        checkTADisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0));
        checkTVDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
        getDistanceFromTargetDisplay.setDouble(getDistanceFromTarget());
        turretPos.setDouble(getPwmPosition());
        turretRotationMin.setDouble(getPwmPosition() - 307);
        turretRotationMax.setDouble(getPwmPosition() + 307);
    }

    public double getTx() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }

    public boolean targetVisible() {
        // check Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tv = table.getEntry("tv");
        double v = tv.getDouble(0.0);

        if (v == 1) {
            return true;
        }
        return false;
    }

    public double getTa() {
        // checks the area visible of the Target (0% of image to 100% of image)
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }

    public double getTy() {
        // checks Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }

    public double getDistanceFromTarget() {
        // TODO use this to get distance from target (only while target is visible)
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(getTy())));
        return limeLightDistance;
    }

    public void limeLightOn() {
        // turns the limelight on using defaulvalue
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(defaultvalue);
    }

    public void limeLightStop() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(0);
    }

    public void putTargetAcquired() {
        boolean status;
        // true or false value
        if (getTa() >= taTolerance) {
            status = true;
        } else {
            status = false;
        }
        targetStatus.setBoolean(status);
    }

    public int getPwmPosition() {
        int raw = turret.getSensorCollection().getPulseWidthRiseToFallUs();
        if (raw == 0) {
            int lastValue = this.lastValue;
            if (lastValue == Integer.MIN_VALUE) {
                return 0;
            }
            return lastValue;
        }

        int actualValue = Math.min(4096, raw - 128);
        lastValue = actualValue;
        return actualValue;
    }

    public void turretPower(double output){
        turret.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void turretStop(){
        turret.set(TalonSRXControlMode.PercentOutput, Configrun.get(0, "turretStop"));
    }

    public void rotateTurret(double output) {

            if(getPwmPosition() >= Configrun.get(943, "turretMin") && output > 0) {
                turretPower(output);
            }
            else if(getPwmPosition() <= Configrun.get(1557, "turretMax") && output < 0) {
                turretPower(output);
            }
            else {
                turretStop();
            }
        // else {
        //     // goToZero();
        //     turretStop();
        // }
    }

    public void targeting() {
        double output;
        if(targetVisible()) {
            output = limeLightPid.calculate(getTx(), 0);
            //converts range to % power
            output = output / LIMELIGHT_RANGE;
            if (output < 0) {
               output = output - staticFeedforward;
            }
            else {
                output = output + staticFeedforward;
            }    

        }
        else {
            output = turretPid.calculate(getPwmPosition(), Configrun.get(1250, "turretZero"));
            //converts range to % power
            output = output / TURRET_RANGE;
            if (output < 0) {
                output = output - staticFeedforward;
            }
            else {
                output = output + staticFeedforward;
            }
        }
        rotateTurret(output);
        putValuesToShuffleboard();
    }

    public void turretToZero() {
        double output = turretPid.calculate(getPwmPosition(), Configrun.get(1250, "turretZero"));
        //converts range to % power
        output = output / TURRET_RANGE;
        if (output < 0) {
            output = output - staticFeedforward;
        }
        else {
            output = output + staticFeedforward;
        }

        rotateTurret(output);
        putValuesToShuffleboard();
    }

    // public void goToZero() {
    //     System.out.println("GOING TO ZERO!!!!!!!!!!");
    //     if (getPwmPosition() > Configrun.get(1250, "turretZero") + 500) {
    //         turretPower(-0.2);
    //     }

    //     else if (getPwmPosition() < Configrun.get(1250, "turretZero") - 500) {
    //         turretPower(0.2);
    //     }

    //     else {
    //         turretStop();
    //     }
    // }
}
