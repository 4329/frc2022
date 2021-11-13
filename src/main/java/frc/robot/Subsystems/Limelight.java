package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

//Code here is input from 2020 code and modified for photonvision OS on limelight so angles and distances are left in inches
  /**
   * The Limelight class uses a static method to call the functions, because there is only 1 limelight PC and it always outputs to the same Network 
   * table the Limelight functions should be called without creating an instance of the object, IE Limelight.tx();
   */
public class Limelight {
    //Output Network Table from the limelight with photonvision OS defined so values can be read by the robot code.
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision"); 

    /**
    * Adds the azimuthal angle to the limelight target yaw angle. Then converts to radians before returning value
    *
    * @return the yaw angle in radians.
    */
    public static double tx() {
        return Math.toRadians(VisionConstants.kAzimuthalAngle + table.getEntry("limelight/targetYaw").getDouble(0.0)); 
    }
    /**
    * Adds the eelvation offset to the limelight target pitch angle. Then converts to radians before returning value
    *
    * @return the pitch angle in radians.
    */
    public static double ty() {
        return Math.toRadians(VisionConstants.kElevationOffset + table.getEntry("limelight/targetPitch").getDouble(0.0));
    }
    /**
    * Singifies if the limelight currently has an accetable target, defaulting to false if no value is provided in the Network Table.
    * This default is important so that if the limelight becomes disconnected or gives bad numbers the code will assume there is not a valid target
    *
    * @return true if an acceptable target is visible.
    */
    public static boolean valid() {
        return table.getEntry("limelight/hasTarget").getBoolean(false);
    }

    /**
    * Allows retreival of the target area. 
    *
    * @return the area of the target.
    */
    public static double ta() {
        return table.getEntry("limelight/targetArea").getDouble(0.0);
    }

    /**
    * Calculates the target distance from the limelight in inches using simple trigonometry. For a right triangle it is known
    * the tangent of the non-right angles is equivalent to the opposite side divided by the side adjacenet to the angle that is not
    * the hypotenuse. Algebraically speaking TAN(theta) = O/A where theta is the angle and O and A are the opposite and adjacent sides respectively.
    * When solved algebraically for the adjacent side A which is the distance to the target in this instance. The formula is A = O/TAN(theta)
    * This function uses this priciple to calculate the distance to the target (adjacent) simply using the height (opposite) and the target
    * pitch (theta).
    *
    * @return the distance to the target in inches
    */
    public static double getDistance() {
        return VisionConstants.kTargetCenterHeightFromLens / Math.tan(ty()); 
    }

    /**
    * Enables the limelight LED array. It is important to only enable when in use to comply with FRC Game Rules.
    */
    public static void enable() {
        table.getEntry("ledMode").setNumber(1);
    }
    /**
    * Disables the limelight LED array. It is important to disable when not in use to comply with FRC Game Rules.
    */
    public static void disable() {
        table.getEntry("ledMode").setNumber(0);
    }
}