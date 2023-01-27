package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight {

    private NetworkTable table;
    // x,y,z,rx,ry,rzzz in meters 
    private double[] blankArray = {0.0,0.0,0.0,0.0,0.0,0.0};

    double x;
    double y;
    double area;
    int id;
    double seenTarget;
    double[] botPos;
    private double[][] tagPositions = {
        {7.24310, -2.93659},
        {7.24310, -1.26019},
        {7.24310, 0.41621},
        {7.90832, 2.74161},
        {-7.90832, 2.74161},
        {-7.24310, 0.41621},
        {-7.24310, -1.26019},
        {-7.24310, -2.93659}
    };


    Limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    void GetValues() {

        //Turn off limelight LEDs
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);

        //read values periodically
        x = table.getEntry("tx").getDouble(0.0);
        id = (int)(table.getEntry("tid").getDouble(0));
        y = table.getEntry("ty").getDouble(0.0);
        area = table.getEntry("ta").getDouble(0.0);
        seenTarget = table.getEntry("tv").getDouble(0.0);
        botPos = table.getEntry("botpose").getDoubleArray(blankArray);

        //post to smart dashboard
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightSeenTarget", seenTarget);
        SmartDashboard.putNumberArray("LimelightPose", botPos);
        SmartDashboard.putNumber("TagID",id);

    }

    double[] GetDistanceToTag() {
        // returns the distance to the tag in x,y (meters), z(rotation)
        double[] distances = {tagPositions[id-1][0]-botPos[0],tagPositions[id-1][1]-botPos[1],-x};

        return distances;

    }

}