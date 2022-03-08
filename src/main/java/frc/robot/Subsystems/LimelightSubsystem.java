package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
//import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
    SmartDashboard table;
    SmartDashboard ledSmartDashboard;
    int defaultvalue = 1;
    PIDController limeLightPid;
    double h1In = Configrun.get(36, "h1In"); //height limelight from ground, 36in
    double h2In = Configrun.get(104, "h2In"); //height target, 104in
    double a1Degree = Configrun.get(0.0, "a1Degree"); //limelight angle
    double limeLightDistance;
    int limeLightTolerance = Configrun.get(1, "limeLightTolerance");
    double limelightP = Configrun.get(1.2, "limelightP");
    double limelightI = Configrun.get(0, "limelightI");
    double limelightD = Configrun.get(0.125, "limelightD");
    double taTolerance;
    public double currentDistance = 120;

    private NetworkTableEntry checkTXDisplay;
    private NetworkTableEntry checkTYDisplay;
    private NetworkTableEntry checkTADisplay;
    private NetworkTableEntry getDistanceFromTargetDisplay;
    NetworkTableEntry targetStatus;

    public LimelightSubsystem() {
        limeLightPid = new PIDController(limelightP, limelightI, limelightD);
        taTolerance = Configrun.get(0.3, "taTolerance");
        limeLightPid.setTolerance(limeLightTolerance);
    }

    public void putValuesToShuffleboard() {
        checkTXDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
        checkTYDisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
        checkTADisplay.setDouble(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0));
        getDistanceFromTargetDisplay.setDouble(getDistanceFromTarget());
    }

    public double checkTx() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }
    // check Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)

    public double checkTa() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }
    // checks the area visible of the Target (0% of image to 100% of image)

    public double checkTy() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }
    // checks Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5
    // degrees)

    public void putDistance() {
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(checkTy())));
        // puts the distance from the limelight on smartdashboard
    }

    public double getDistanceFromTarget() {
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(checkTy())));
        return limeLightDistance;
    }

    public void limeLight() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry limeLightDistance = table.getEntry("LimeDis");

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(defaultvalue);
        
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double distance = limeLightDistance.getDouble(0.0);


    }

    public void limeLightStop() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(0);
    }

    public void putTargetAcquired() {
        boolean status;
        if (checkTa() >= taTolerance) {
            status = true;
        } else {
            status = false;
        }
        targetStatus.setBoolean(status);
    }

}