package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configrun;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class LimelightSubsystem extends SubsystemBase {
    SmartDashboard table;
    SmartDashboard ledSmartDashboard;
    //default value for the limelight mode
    int defaultvalue = 1;
    PIDController limeLightPid;
    double h1In = Configrun.get(36, "h1In");
    //height of the limelight off of the ground 36 inches
    double h2In = Configrun.get(104, "h2In");
    //height of the target 8ft 8inches aka 104 inches
    double a1Degree = Configrun.get(0.0, "a1Degree");
    double limeLightDistance;
    int limeLightTolerance = Configrun.get(1, "limeLightTolerance");
    double limelightP = Configrun.get(1.2, "limelightP");
    double limelightI = Configrun.get(0, "limelightI");
    double limelightD = Configrun.get(0.125, "limelightD");
    double staticFeedforward = Configrun.get(0.103, "turnStaticFeedforward");
    NetworkTableEntry targetStatus;
    double taTolerance;
    public double currentDistance = 120;

    public LimelightSubsystem() {
        limeLightPid = new PIDController(limelightP, limelightI, limelightD);
        limeLightPid.setTolerance(limeLightTolerance);
        targetStatus = Shuffleboard.getTab("TestValues").add("Target Acquired", false).getEntry();
        taTolerance = Configrun.get(0.3, "taTolerance");
    }

    public void setDistance() {
        currentDistance = getDistanceFromTarget();
    }
    // Sets calculation distance for shooter

    public double checkTx() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        double x = tx.getDouble(0.0);
        return x;
    }
    // check Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)

    // public void runLimelightPID() {
    //     if (!RobotContainer.shooter.isShooterOverriden()) {
    //         double output = limeLightPid.calculate(checkTx(), 0);
    //         output = output / 30;
    //         if (output < 0) {
    //             output = output - staticFeedforward;
    //         } else {
    //             output = output + staticFeedforward;
    //         }
    //         // SmartDashboard.putNumber("LimelightOutput", output);

    //         RobotContainer.swerveDrive.drive(0, 0, output);
    //         setDistance();

    //     }
    // }
    //need a shooter code for the code above to be needed

    public double checkTa() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ta = table.getEntry("ta");
        double a = ta.getDouble(0.0);
        return a;
    }
    // checks the area visible of the Target (0% of image to 100% of image)

    private double checkTy() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double y = ty.getDouble(0.0);
        return y;
    }
    //checks Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)

    public void putDistance() {
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(checkTy())));
        // runing this math equation d = (h2-h1) / tan(a1+a2)
        // a2 = Ty
        // Ty = vertical offset
        SmartDashboard.putNumber("Limelight Distance", limeLightDistance);
        //puts the distance from the limelight on smartdashboard
    }

    public double getDistanceFromTarget() {
        limeLightDistance = (h2In - h1In) / Math.tan(Math.toRadians(a1Degree) + (Math.toRadians(checkTy())));
        return limeLightDistance;
    }
    // limelight distance equation
    // d = (h2-h1) / tan(a1+a2)

    public void limeLight() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry limeLightDistance = table.getEntry("Limelight Distance");
        // NetworkTableEntry ledSmartDashboard = table.getEntry("ledSmartDashboard");


        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(defaultvalue);
        //turns the limelight on using defaulvalue

        double x = tx.getDouble(0.0);
        // read values of tx to convert to x periodically
        double y = ty.getDouble(0.0);
        // read values of ty to convert to y periodically
        double area = ta.getDouble(0.0);
        // read values of ta to convert to area periodically
        double distance = limeLightDistance.getDouble(0.0);
        // read values of ta to convert to area periodically
        
        SmartDashboard.putNumber("LimelightX", x);
        // post the value of x to smart dashboard periodically
        SmartDashboard.putNumber("LimelightY", y);
        // post the value of y to smart dashboard periodically
        SmartDashboard.putNumber("LimelightArea", area);
        // post the value of area to smart dashboard periodically

    }

    public void limeLightStop() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(0);
    }

    public void putTargetAcquired() {
        boolean status;
        //true or false value
        if (checkTa() >= taTolerance) {
            status = true;
        } else {
            status = false;
        }
        targetStatus.setBoolean(status);
    }
}