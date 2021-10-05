package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static final double ELEVATION_OFFSET = 15.0; //Degree offset of lens from horizontal due to camera mount
    private static final double AZIMUTHAL_OFFSET = 0.5;
    private static final double LENS_TO_TARGET_HEIGHT   = 63.25; //Distance from center of bounding box to lens: lens to outer port bottom edge = 55.75; outer port bottom edge to center of bounding box = 7.5


    public static double tx() {
        return  (AZIMUTHAL_OFFSET + table.getEntry("tx").getDouble(0.0))*Math.PI/180.0;
    }

    public static double ty() {
        return  ELEVATION_OFFSET + table.getEntry("ty").getDouble(0.0);
    }

    public static boolean valid() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public static double ta(){
        return table.getEntry("ta").getDouble(0.0);
    }

    public static double getDistance() {
        return LENS_TO_TARGET_HEIGHT/Math.tan(Math.toRadians(ty()));
    }

    public static void enable() {
        table.getEntry("ledMode").setNumber(0);
    }

    public static void disable() {
        table.getEntry("ledMode").setNumber(1);
    }
}