package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {

    private static PathPlannerTrajectory selectedPath;
    public static Command autoCommand;

    private static final double maxVel = 4;
    private static final double maxAccel = 3;

    private static PIDController linearController = new PIDController(0, 0, 0);
    private static PIDController rotationalController = new PIDController(0, 0, 0);

    Auto() {

    }

    public static void selectAuto(String autoSequence, Supplier<Pose2d> input, Consumer<ChassisSpeeds> output) {
        selectedPath = PathPlanner.loadPath(autoSequence, new PathConstraints(maxVel, maxAccel));
        autoCommand = new PPSwerveControllerCommand(selectedPath, input, linearController, linearController, rotationalController, output);
    }

}