package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;

import java.awt.geom.Point2D;
import frc.robot.Utilities.LinearInterpolationTable;

public class TurretSubsystem extends SubsystemBase{

    private static final double LIMELIGHT_RANGE = 30;
    private static final double TURRET_RANGE = 4128;

    private TalonSRX turret;
    private volatile int lastValue = Integer.MIN_VALUE;

    double staticFeedforward = 0;
    double zeroStaticFeedforward = 0.01;
    // default value for the limelight mode
    int defaultvalue = 1;
    PIDController limeLightPid;
    PIDController turretPid;
    double h1In = Configrun.get(30.5, "h1In");
    // height of the limelight off of the ground "36 inches" this year
    double h2In = Configrun.get(104, "h2In");
    // height of the target "8ft 8inches" aka 104 inches this year
    double a1Degree = Configrun.get(25, "a1Degree");
    // angle of the front of the limelight in relation to level
    double limeLightDistance;
    int limeLightTolerance = 1;
    int turretTolerance = 2;
    double taTolerance = 0.3;
    final double TURRET_MIN = Configrun.get(2347, "turretMin");
    final double TURRET_MAX = Configrun.get(2961, "turretMax");
    final double TURRET_ZERO = Configrun.get(2654, "turretZero");


    public double currentDistance = 120;

    private NetworkTableEntry checkTXDisplay;
    private NetworkTableEntry checkTYDisplay;
    private NetworkTableEntry checkTADisplay;
    private NetworkTableEntry checkTVDisplay;
    private NetworkTableEntry getDistanceFromTargetDisplay;
    private NetworkTableEntry turretPos;
    private NetworkTableEntry turretRotationMin;
    private NetworkTableEntry turretRotationMax;
    private boolean tvToggle;
    NetworkTableEntry targetStatus;

    private Point2D[] limlightTable = new Point2D.Double[] {


        new Point2D.Double(-14.93, 276),
        new Point2D.Double(-14.11, 264),
        new Point2D.Double(-12.75, 240),
        new Point2D.Double(-10.12, 210),
        new Point2D.Double(-8.37, 186),
        new Point2D.Double(-6.67, 174),
        new Point2D.Double(-4.44, 156),
        new Point2D.Double(-1.547, 138),
        new Point2D.Double(1.999, 120),
        new Point2D.Double(7.169, 102),
        new Point2D.Double(13.8, 83),
        new Point2D.Double(21, 67)

    };
    private LinearInterpolationTable m_limlightTable = new LinearInterpolationTable(limlightTable);

    public TurretSubsystem() {
        turret = new TalonSRX (Configrun.get(41, "turretID"));
        limeLightPid = new PIDController(1, 0, 0);
        limeLightPid.setTolerance(limeLightTolerance);
        turretPid = new PIDController(6.5, 0, 0);
        turretPid.setTolerance(turretTolerance);

        checkTVDisplay = Shuffleboard.getTab("RobotData").add("Target Visible", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 2).getEntry();
        getDistanceFromTargetDisplay = Shuffleboard.getTab("RobotData").add("Distance", 0).withPosition(2, 2).getEntry();

        if (Configrun.get(false, "extraShuffleBoardToggle")) {
            targetStatus = Shuffleboard.getTab("Limlight").add("Target Acquired", false).getEntry();
            checkTXDisplay = Shuffleboard.getTab("Limlight").add("TX", 0).withPosition(3, 1).getEntry();
            checkTYDisplay = Shuffleboard.getTab("Limlight").add("TY", 0).withPosition(3, 0).getEntry();
            checkTADisplay = Shuffleboard.getTab("Limlight").add("TA", 0).withPosition(4, 0).getEntry();
            turretPos = Shuffleboard.getTab("Limlight").add("Turret Position", getPwmPosition()).withPosition(3, 2).getEntry();
            turretRotationMin = Shuffleboard.getTab("Limlight").add("Find Turret Minimum", getPwmPosition() - 307).withPosition(3, 3).getEntry();
            turretRotationMax = Shuffleboard.getTab("Limlight").add("Find Turret Maximum", getPwmPosition() + 307).withPosition(4, 2).getEntry();
        }
    }

    public void putValuesToShuffleboard() {
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
            tvToggle = true;
        }
        else {
            tvToggle = false;
        }

        checkTVDisplay.setBoolean(tvToggle);
        getDistanceFromTargetDisplay.setDouble(getDistanceFromTarget());

        if (Configrun.get(false, "extraShuffleBoardToggle")){
            checkTXDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
            checkTYDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
            checkTADisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0));
            turretPos.setDouble(getPwmPosition());
            turretRotationMin.setDouble(getPwmPosition()- 307);
            turretRotationMax.setDouble(getPwmPosition()+ 307);
        }
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
       return m_limlightTable.getOutput(getTy());
        //limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(getTy())));
       // return limeLightDistance;
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

        int actualValue = Math.min(4128, raw );
        lastValue = actualValue;
        return actualValue;
    }

    public void turretPower(double output)
    {
       turret.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void turretStop(){
        turret.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void rotateTurret(double output) {

        double pwmPos = getPwmPosition();

        if(pwmPos == 0) {
            turretStop();
        }
        else if(pwmPos >= TURRET_MIN && output > 0) {
            turretPower(output);
        }
        else if(pwmPos <= TURRET_MAX && output < 0) {
            turretPower(output);
        }
        else {
            turretStop();
        }
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
            rotateTurret(output);
        }
        else {

            turretStop();
        }

        putValuesToShuffleboard();
    }

    public void turretToZero(/*double toZero*/) {

        double encoderReading = getPwmPosition();
        if (encoderReading < TURRET_MAX + 500 && encoderReading > TURRET_MIN - 500) {

            double output = turretPid.calculate(encoderReading, TURRET_ZERO);
            //converts range to % power
            output = output / TURRET_RANGE;

            if (output < 0) {
                output = output - zeroStaticFeedforward;
            }
            else {
                output = output + zeroStaticFeedforward;
            }
            turretPower(-1 * output);
            putValuesToShuffleboard();
        }
        else {
            turretStop();
        }
    }

    public boolean targeted() {

        return limeLightPid.atSetpoint();
    }

}
