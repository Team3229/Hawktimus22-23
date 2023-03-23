//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leveling {
    private static final double MOVE_SPEED = 0.1;
    private static final double ANGLE_TOLERANCE = 0.27;

    static Dashboard dash = new Dashboard();
    Leveling() {}

    public static double getBalanced(double currentPitch) {

        SmartDashboard.putNumber("pitch", currentPitch+0.2599);

        if (currentPitch+0.2599 < -ANGLE_TOLERANCE) {
            return -MOVE_SPEED;
        } else if (currentPitch+0.2599 > ANGLE_TOLERANCE) {
            return MOVE_SPEED;
        } else {
            return 0;
        }
    }
}