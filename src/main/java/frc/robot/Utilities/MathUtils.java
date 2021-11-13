package frc.robot.Utilities;

  /**
   * This file is created to hold any particularly useful math utilies which may be called statically
   */
public class MathUtils {
    //Converts a radian angle into the unit circle range of 0 to 2PI
    /**
    * This function takes in an angle in radians and does the proper math to convert the output to an angle from 0 to 2PI
   * also known as a "unit circle angle"
    *
    * @param angle is the input angle in radians
    * @return a unit circle angle equal to the input angle 
    */
    public static double toUnitCircAngle(double angle){
        double rotations = angle / (2*Math.PI);        
        return (angle - Math.round(rotations-0.500)*Math.PI*2.0);
    }

}
