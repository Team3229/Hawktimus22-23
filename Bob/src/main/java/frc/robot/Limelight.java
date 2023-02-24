//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;

public class Limelight {
    private final NetworkTable table;
    PIDController xAlignPIDController;
    PIDController yAlignPIDController;
    PIDController zAlignPIDController;
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

    double x;
    double y;
    double area;
    int id;
    double seenTarget;
    double[] botPos;
    double[] xAlignPID = {0, 0, 0};
    double[] yAlignPID = {0, 0, 0};
    double[] zAlignPID = {0, 0, 0};

    Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        xAlignPIDController = new PIDController(xAlignPID[0], xAlignPID[1], xAlignPID[2]);
        yAlignPIDController = new PIDController(yAlignPID[0], yAlignPID[1], yAlignPID[2]);
        zAlignPIDController = new PIDController(zAlignPID[0], zAlignPID[1], zAlignPID[2]);
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
        double[] distances = {tagPositions[id - 1][0] - botPos[0], tagPositions[id - 1][1] - botPos[1], -x, id};

        return distances;
    }

    double[] alignWithTag() {
        // 1.2192 meters is the minimum distance to be, that's how far from the tag we want to be.

        double[] distanceToTag = getDistanceToTag();
        double[] actualMove = {distanceToTag[2]};

        return actualMove;
    }
}