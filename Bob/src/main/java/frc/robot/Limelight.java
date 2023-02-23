package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class Limelight {
    private final NetworkTable table;
    private final double[] blankArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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

    Dashboard dash = new Dashboard();

    public double x;
    public double y;
    public double area;
    public int id;
    public double seenTarget;
    public double[] botPos;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void updateDashboard() {
        dash.putNumber("LimelightX", x);
        dash.putNumber("LimelightY", y);
        dash.putNumber("LimelightArea", area);
        dash.putNumber("LimelightSeenTarget", seenTarget);
        dash.putNumberArray("LimelightPose", botPos);
        dash.putNumber("TagID", id);
    }

    public void getValues() {
        // Turn off Limelight LEDs
        table.getEntry("ledMode").setNumber(1);

        // Read values periodically
        x = table.getEntry("tx").getDouble(0.0);
        id = (int) table.getEntry("tid").getDouble(0);
        y = table.getEntry("ty").getDouble(0.0);
        area = table.getEntry("ta").getDouble(0.0);
        seenTarget = table.getEntry("tv").getDouble(0.0);
        botPos = table.getEntry("botpose").getDoubleArray(blankArray);

        updateDashboard();
    }

    public double[] getDistanceToTag() {
        // Returns the distance to the closest tag in x, y (meters), z(rotation)
        double[] distances = {tagPositions[id - 1][0] - botPos[0], tagPositions[id - 1][1] - botPos[1], -x, id};

        return distances;
    }

    public double[] alignWithTag() {
        // 1.2192 meters is the minimum distance to be, that's how far from the tag we want to be.
        final double MIN_DISTANCE = 1.2192;
        final double ALIGN_SCALE_FACTOR = 0.05;

        double[] distanceToTag = getDistanceToTag();
        double[] actualMove = {0, distanceToTag[1] * ALIGN_SCALE_FACTOR, distanceToTag[2]};
        
        if (Math.abs(distanceToTag[0]) > MIN_DISTANCE) {
            actualMove[0] = distanceToTag[0] * ALIGN_SCALE_FACTOR;
        }

        return actualMove;
    }
}