package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leveling {


    Leveling() {

    }

    static double GetBalanced(double currentAngle) {

        SmartDashboard.putNumber("navX-roll", currentAngle);
        double moveSpeed = 0.1;
        double totalAngle = 10;
        if(currentAngle < -totalAngle){
            // move forward
            return moveSpeed;
        } else if (currentAngle > totalAngle){
            // backwards
            return -moveSpeed;
        } else {
            // nothing
            return 0;
        }
        
    }

    
}
