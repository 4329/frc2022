package frc.robot.Utilities;

public class MathUtils {
    //Converts a radian angle into the unit circle range of 0 to 2PI
    public static double toUnitCircAngle(double angle){
        double rotations = angle / (2*Math.PI);        
        return (angle - Math.round(rotations-0.500)*Math.PI*2.0);
    }

}
