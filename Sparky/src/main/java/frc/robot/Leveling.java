//Otters: 3229 Programming SubTeam

package frc.robot;
public class Leveling {
    private static final double MOVE_SPEED = 0.1;
    private static final double MOUNTING_SPEED = 0.4;
    private static final double ANGLE_TOLERANCE = 5;

    static Dashboard dash = new Dashboard();
    Leveling() {}

    public static double getBalanced(double currentPitch, boolean onChargeStation) {

        if (!onChargeStation) {
            return -MOUNTING_SPEED;
        }

        if (currentPitch < -ANGLE_TOLERANCE) {
            return -MOVE_SPEED;
        } else if (currentPitch > ANGLE_TOLERANCE) {
            return MOVE_SPEED;
        } else {
            return 0;
        }
    }
}