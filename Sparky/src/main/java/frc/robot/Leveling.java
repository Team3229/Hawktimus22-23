//Otters: 3229 Programming SubTeam

package frc.robot;

public class Leveling {
    private static final double MOVE_SPEED = 0.5;
    private static final double ANGLE_TOLERANCE = 0.2;
    public static final double PITCH_OFFSET = 0.25;

    Leveling() {}

    public static double getBalanced(double currentPitch) {

        if (currentPitch < -ANGLE_TOLERANCE | currentPitch > ANGLE_TOLERANCE) {
            // return MOVE_SPEED*currentPitch;
            return 0;
        } else {
            return 0;
        }
    }
}