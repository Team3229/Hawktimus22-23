//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;

public class Limelight {
    private final NetworkTable table;
    private final double MIN_DISTANCE = 1.2192-0.4;
    private final double[][] tagPositions = {
            {7.24310-MIN_DISTANCE, -2.93659},
            {7.24310-MIN_DISTANCE, -1.26019},
            {7.24310-MIN_DISTANCE, 0.41621},
            {7.90832-MIN_DISTANCE, 2.74161},
            {-7.90832+MIN_DISTANCE, 2.74161},
            {-7.24310+MIN_DISTANCE, 0.41621},
            {-7.24310+MIN_DISTANCE, -1.26019},
            {-7.24310+MIN_DISTANCE, -2.93659}
    };
    private final double MOVE_SPEED = 0.3;
    private final double ROTATE_SPEED = -0.02;
    private final double MOVE_TOLERANCE = 0.03;
    private final double ROTATE_TOLERANCE = 0.015;

    Translation2d location;
    Rotation2d rotation;
    double area;
    int id;
    double seenTarget;
    double[] botPos;

    Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("stream").setInteger(2);
    }

    void getValues() {
        // Turn off Limelight LEDs
        table.getEntry("ledMode").setNumber(1);

        // Read values periodically
        id = (int) table.getEntry("tid").getDouble(0);
        seenTarget = table.getEntry("tv").getDouble(0.0);
        botPos = table.getEntry("botpose").getDoubleArray(new double[6]);
        location = new Translation2d(botPos[0], botPos[1]);
        rotation = new Rotation2d(Utils.convertDegreesToRadians(botPos[5]));
        
    }

    double[] getDistanceToTag() {
        // Returns the distance to the targeted tag in x, y (meters), z(rotation)
        if (id != -1) {
            return new double[] {(tagPositions[id-1][0] - location.getX()), tagPositions[id-1][1] - location.getY(), 0};
        } else {
            return new double[3];
        }
    }

    double[] alignWithTag(double currentChassisAngle, Alliance alliance) {
        // MIN_DISTANCE meters is the minimum distance to be, that's how far from the tag we want to be.

        double[] distanceToTag = getDistanceToTag();

        if (alliance == Alliance.Blue) {
            distanceToTag[0] = -distanceToTag[0];
            distanceToTag[1] = -distanceToTag[1];
        }
        
        
        distanceToTag[2] = rotation.minus(Rotation2d.fromDegrees(currentChassisAngle)).getDegrees();

        //Move tolerances
        distanceToTag[0] = (Math.abs(distanceToTag[0]*MOVE_SPEED) > MOVE_TOLERANCE) ? 0 : distanceToTag[0];
        distanceToTag[1] = (Math.abs(distanceToTag[1]*MOVE_SPEED) > MOVE_TOLERANCE) ? 0 : distanceToTag[1];
        distanceToTag[2] = (Math.abs(distanceToTag[2]*ROTATE_SPEED) > ROTATE_TOLERANCE) ? 0 : distanceToTag[2];

        return new double[] {distanceToTag[1]*MOVE_SPEED, distanceToTag[0]*MOVE_SPEED, distanceToTag[2]*ROTATE_SPEED};
    }

}