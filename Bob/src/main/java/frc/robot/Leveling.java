package frc.robot;
public class Leveling {
    private static final double MOVE_SPEED = 0.1;
    private static final double TOTAL_ANGLE = 10;

    static Dashboard dash = new Dashboard();
    Leveling() {}

    public static double getBalanced(double currentAngle) {

        dash.putNumber("navX-roll", currentAngle);

        if (currentAngle < -TOTAL_ANGLE) {
            return MOVE_SPEED;
        } else if (currentAngle > TOTAL_ANGLE) {
            return -MOVE_SPEED;
        } else {
            return 0;
        }
    }
}