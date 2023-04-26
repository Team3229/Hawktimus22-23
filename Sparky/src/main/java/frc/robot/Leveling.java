//Otters: 3229 Programming SubTeam

package frc.robot;

/** Auto-Leveling class
 * @deprecated Does not work
 */
public class Leveling {
    private static final double ANGLE_TOLERANCE = 0.2;
    public static final double PITCH_OFFSET = 0.25;

    Leveling() {}

    /**
     * Balance on the charging station, must be called periodically
     * @param currentPitch (double) The current pitch of the robot
     * @return (double) The move speed to get balanced
     */
    public static double getBalanced(double currentPitch) {

        if (currentPitch < -ANGLE_TOLERANCE | currentPitch > ANGLE_TOLERANCE) {
            // return MOVE_SPEED*currentPitch;
            return 0;
        } else {
            return 0;
        }
    }
}