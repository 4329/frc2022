package frc.robot.utils;

public class MathUtils {
 
    public static double toUnitCircAngle(double angle){
        double rotations = angle / (2*Math.PI);

        angle = angle - Math.round(rotations-0.500)*Math.PI*2;
        
        return angle;
    }

}
