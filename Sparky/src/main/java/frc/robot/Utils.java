//Otters: 3229 Programming Sub-Team

package frc.robot;

/**
 * A utility class for common calculations and conversions.
 */
public class Utils {
    
    /**
     * Converts from meters per second to revolutions per minute, given the wheel radius.
     * @param speedMps The speed in meters per second.
     * @param wheelRadiusM The radius of the wheel in meters.
     * @return The speed in revolutions per minute.
     */
    public static double convertMpsToRpm(double speedMps, double wheelRadiusM) {
        return (60 / (2 * Math.PI) * wheelRadiusM) * speedMps;
    }

    public static double convertDegreesToRadians(double degrees) {
        return (degrees * Math.PI/180);
    }

    /**
     * Gets directional pad values based on the input POV angle.
     * @param povAngle The angle of the POV (in degrees).
     * @return An array containing the x, y, and z values of the directional pad.
     */
    public static double[] getDirectionalPadValues(int povAngle) {
        double[] output = {0, 0, 0};
        switch (povAngle) {
            case 0:
                output[0] = 0; output[1] = -0.5;
                break;
            case 45:
                output[0] = 0.4; output[1] = -0.4;
                break;
            case 90:
                output[0] = 0.5; output[1] = 0;
                break;
            case 135:
                output[0] = 0.4; output[1] = 0.4;
                break;
            case 180:
                output[0] = 0; output[1] = 0.5;
                break;
            case 225:
                output[0] = -0.4; output[1] = 0.4;
                break;
            case 270:
                output[0] = -0.5; output[1] = 0;
                break;
            case 315:
                output[0] = -0.4; output[1] = -0.4;
                break;
          }
          return output;
    }
}
