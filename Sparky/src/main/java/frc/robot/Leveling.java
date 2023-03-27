//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leveling {
    private static final double MOVE_SPEED = -0.2;
    private static final double ANGLE_TOLERANCE = 0.3;
    private static final double PITCH_OFFSET = 0.2599;

    Leveling() {}

    public static double getBalanced(double currentPitch) {
        
        currentPitch += PITCH_OFFSET;

        SmartDashboard.putNumber("pitch", currentPitch);

        if (currentPitch < -ANGLE_TOLERANCE) {
            return -MOVE_SPEED;
        } else if (currentPitch > ANGLE_TOLERANCE) {
            return MOVE_SPEED;
        } else {
            return 0;
        }
    }
}