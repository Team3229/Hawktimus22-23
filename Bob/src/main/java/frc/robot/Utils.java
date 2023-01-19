// Otter: Nathan Manhardt (3229 Software Captain)
// Co-Otters: 3229 Software Team

package frc.robot;


public class Utils {
    // in meters
    Utils() {}
    
    /**
     * Converts from any, positive or negative, angle to the range of 0-360
     * @param input Input angle
     * @return Output angle
     */
    double convertAngle(double input) {

        return (input)-(360*Math.floor((input)/360));

    }
    /**
     * Converts from Meters Per Second to RPM (wheel radius already included)
     * @param input Input m/s
     * @return Output RPM
     */
    double mpsToRPM(double input) {

        return (60/(2*Math.PI)*0.0508) * input;

    }

    double[] dirPad(int POV) {
        double[] output = {0, 0, 0};
        switch (POV) {
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
          }

          return output;
    }

}
