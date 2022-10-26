package frc.robot.Subsystems;

import java.awt.geom.Point2D;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import frc.robot.Constants;
import frc.robot.Utilities.LinearInterpolationTable;

public class TurretSubsystem extends SubsystemBase{

    private static final double LIMELIGHT_RANGE = 70;
    private static final double TURRET_RANGE = 70;

    private CANSparkMax turret;
    private RelativeEncoder turretEncoder;

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
    final double TURRET_MIN = Configrun.get(-35, "turretMin");
    final double TURRET_MAX = Configrun.get(35, "turretMax");
    final double TURRET_ZERO = Configrun.get(0, "turretZero");


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

    public TurretSubsystem() {

        turret = new CANSparkMax(Configrun.get(43/*29*/, "turretID"), MotorType.kBrushless);
        turretEncoder = turret.getEncoder();
        turret.setIdleMode(IdleMode.kBrake);
        turretEncoder.setPosition(0);
        limeLightPid = new PIDController(6.5, 0, 0);
        limeLightPid.setTolerance(limeLightTolerance);
        turretPid = new PIDController(6.5, 0, 0);
        turretPid.setTolerance(turretTolerance);

        checkTVDisplay = Shuffleboard.getTab("RobotData").add("Target Visible", false).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 2).getEntry();
        getDistanceFromTargetDisplay = Shuffleboard.getTab("RobotData").add("Distance", 0).withPosition(2, 2).getEntry();

        if (Configrun.get(false, "extraShuffleBoardToggle")) {
            targetStatus = Shuffleboard.getTab("Limlight").add("Target Acquired", false).getEntry();
            checkTXDisplay = Shuffleboard.getTab("Limlight").add("TX", 0).withPosition(3, 1).getEntry();
            checkTYDisplay = Shuffleboard.getTab("RobotData").add("TY", 0).withPosition(3, 0).getEntry();
            checkTADisplay = Shuffleboard.getTab("Limlight").add("TA", 0).withPosition(4, 0).getEntry();
            turretPos = Shuffleboard.getTab("Limlight").add("Turret Position", getEncoderPosition()).withPosition(3, 2).getEntry();
            turretRotationMin = Shuffleboard.getTab("Limlight").add("Find Turret Minimum", getEncoderPosition() - 35).withPosition(3, 3).getEntry();
            turretRotationMax = Shuffleboard.getTab("Limlight").add("Find Turret Maximum", getEncoderPosition() + 35).withPosition(4, 2).getEntry();
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
            turretPos.setDouble(getEncoderPosition());
            turretRotationMin.setDouble(getEncoderPosition()- 35);
            turretRotationMax.setDouble(getEncoderPosition()+ 35);
        }
    }

    public static double getTx() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }

    public static boolean targetVisible() {

        // check Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tv = table.getEntry("tv");
        double v = tv.getDouble(0.0);

        if (v == 1) {
            return true;
        }
        return false;
    }

    public static double getTa() {

        // checks the area visible of the Target (0% of image to 100% of image)
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }

    public static double getTy() {

        // checks Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }


    public static double getDistanceFromTarget() {

        // TODO use this to get distance from target (only while target is visible)
        double yadjustement = (getTy()-0.006039*getTx()*getTx())/(0.000306*getTx()*getTx()+1);
       return Constants.TuningConstants.m_limlightTable.getOutput(yadjustement);
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

    public double getEncoderPosition() {

        return turretEncoder.getPosition();
    }

    public void turretPower(double output) {

        turret.set(output);
    }

    public void turretStop() {

        turret.set(0);
    }

    public void rotateTurret(double output) {

        double pwmPos = getEncoderPosition();

        if(pwmPos >= TURRET_MIN && pwmPos <= TURRET_MAX) {
            turretPower(output);
        } else {
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
            rotateTurret(-output);
        }
        else {

            turretStop();
        }

        putValuesToShuffleboard();
    }

    public void turretToZero() {

        double encoderReading = getEncoderPosition();
        if (encoderReading < TURRET_MAX + 15 && encoderReading > TURRET_MIN - 15) {

            double output = turretPid.calculate(encoderReading, TURRET_ZERO);
            //converts range to % power
            output = output / TURRET_RANGE;

            if (output < 0) {
                output = output - zeroStaticFeedforward;
            }
            else {
                output = output + zeroStaticFeedforward;
            }
            turretPower(output);
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