//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;

public class Limelight {
    private final NetworkTable table;
    // private final double MIN_DISTANCE = 0.8192;
    public static final double[] tagPositions = {
            -2.9365,
            -1.2601,
            0.4162,
            2.7416,
            2.7416,
            0.4162,
            -1.2601,
            -2.9365
    };
    // private final double MOVE_SPEED = -0.2;
    // private final double ROTATE_SPEED = -0.0005;
    // private final double MOVE_TOLERANCE = 0.2;
    // private final double ROTATE_TOLERANCE = 0.05;

    private final double pos_kP = -0.7;
    private final double pos_kI = 0;
    private final double pos_kD = 0;
    private final double rot_kP = 0.02;
    private final double rot_kI = 0;
    private final double rot_kD = 0;

    private final PIDController posPID = new PIDController(pos_kP, pos_kI, pos_kD);
    private final PIDController rotPID = new PIDController(rot_kP, rot_kI, rot_kD);

    Translation2d location;
    Rotation2d rotation;
    double area;
    public int id;
    double seenTarget;
    double[] botPos;

    Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("stream").setInteger(0);

        rotPID.enableContinuousInput(0, 360);
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

    double goToTag() {
        if (id>-1&id<9) {
            return tagPositions[id-1];
        } else {
            return 0;
        }
    }

    double[] goToTarget(double targetX, double targetZ, Alliance alliance) {
        if (id>-1&id<9) {
            posPID.setSetpoint(targetX);
            rotPID.setSetpoint(0);
            return new double[] {
                ((alliance == Alliance.Red)?-1:1)*posPID.calculate(location.getY()+0.15),
                0,
                rotPID.calculate(targetZ)
            };
        } else {
            return new double[] {0, 0, 0};
        }
    }

}