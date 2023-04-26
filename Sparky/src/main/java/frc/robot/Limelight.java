//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;

/** Limelight class for detecting and using april tags
 */
public class Limelight {
    private final NetworkTable table;

    private final double pos_kP = -0.7;
    private final double pos_kI = 0;
    private final double pos_kD = 0;
    private final double rot_kP = 0.02;
    private final double rot_kI = 0;
    private final double rot_kD = 0;

    private final PIDController posPID = new PIDController(pos_kP, pos_kI, pos_kD);
    private final PIDController rotPID = new PIDController(rot_kP, rot_kI, rot_kD);

    Pose2d position;
    int id;
    boolean seesTag;
    double[] botPos;

    Limelight() {

        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("stream").setInteger(0);

        rotPID.enableContinuousInput(0, 360);
    }

    /**Gets the values from the limelight */
    void getValues() {
        // Turn off Limelight LEDs
        table.getEntry("ledMode").setNumber(1);

        // Read values periodically
        id = (int) table.getEntry("tid").getDouble(0);
        seesTag = (table.getEntry("tv").getDouble(0) != 0 ? true : false);
        botPos = table.getEntry("botpose").getDoubleArray(new double[6]);
        position = new Pose2d(new Translation2d(botPos[0], botPos[1]), Rotation2d.fromDegrees(botPos[5]));
    }

    /**
     * Goes in the direction of the april tag, if there is one in sight
     * @deprecated Does not work
     * @param targetX (double) The april tags X
     * @param targetY (double) The april tags Y
     * @param targetZ (double) The april tags Z
     * @param alliance (Alliance) The alliance the robot is a part of
     * @return (double[]) The movements to get closer to the tag
     */
    double[] goToTarget(double targetX, double targetY, double targetZ, Alliance alliance) {
        if (seesTag) {
            posPID.setSetpoint(targetX);
            rotPID.setSetpoint(targetZ);
            return new double[] {
                ((alliance == Alliance.Red)?-1:1)*posPID.calculate(position.getY()),
                ((alliance == Alliance.Red)?-1:1)*posPID.calculate(position.getX()),
                rotPID.calculate(position.getRotation().getDegrees())
            };
        } else {
            return new double[] {0, 0, 0};
        }
    }

}