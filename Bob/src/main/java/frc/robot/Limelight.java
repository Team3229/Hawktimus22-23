//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class Limelight {
    private final NetworkTable table;
    private final double[][] tagPositions = {
            {7.24310, -2.93659},
            {7.24310, -1.26019},
            {7.24310, 0.41621},
            {7.90832, 2.74161},
            {-7.90832, 2.74161},
            {-7.24310, 0.41621},
            {-7.24310, -1.26019},
            {-7.24310, -2.93659}
    };
    private final double MIN_DISTANCE = 1.2192;
    private final double MOVE_SPEED = 0;
    private final double CONE_OFFSET = 0.561975;
    private int targetedGrid = 0;

    double x;
    double y;
    double area;
    int id;
    double seenTarget;
    double[] botPos;

    Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    void getValues() {
        // Turn off Limelight LEDs
        table.getEntry("ledMode").setNumber(1);

        // Read values periodically
        x = table.getEntry("tx").getDouble(0.0);
        id = (int) table.getEntry("tid").getDouble(0);
        y = table.getEntry("ty").getDouble(0.0);
        area = table.getEntry("ta").getDouble(0.0);
        seenTarget = table.getEntry("tv").getDouble(0.0);
        botPos = table.getEntry("botpose").getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    double[] getDistanceToTag() {
        // Returns the distance to the closest tag in x, y (meters), z(rotation)
        return new double[] {tagPositions[id - 1][0] - botPos[0], tagPositions[id - 1][1] - botPos[1], -x, id};
    }

    double[] alignWithTag(boolean cube) {
        // 1.2192 meters is the minimum distance to be, that's how far from the tag we want to be.

        double[] distanceToTag = getDistanceToTag();
        // offsets for closest 
        if (targetedGrid == 0 & !cube) {
            if (distanceToTag[0] > 0 & !cube) {
                targetedGrid = 1;
            } else if (distanceToTag[0] < 0 & !cube) {
                targetedGrid = 3;
            } else if (!cube) {
                targetedGrid = 1;
            }
        } else if (targetedGrid == 0 & cube){
            targetedGrid = 2;
        } else {
            //do here
            if (targetedGrid == 1) {
                distanceToTag[0] -= CONE_OFFSET;
            }
            if (targetedGrid == 3) {
                distanceToTag[0] += CONE_OFFSET;
            }

            return new double[] {distanceToTag[0]*MOVE_SPEED, (distanceToTag[1]*MOVE_SPEED)-MIN_DISTANCE, distanceToTag[2]*MOVE_SPEED};
        }

        return new double[] {distanceToTag[0]*MOVE_SPEED, (distanceToTag[1]*MOVE_SPEED)-MIN_DISTANCE, distanceToTag[2]*MOVE_SPEED};
    }

}