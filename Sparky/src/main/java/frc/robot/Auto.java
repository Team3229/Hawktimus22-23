package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
	// Capture/Replay
	
	private static final String basePath = "/home/lvuser/";
	public static final String basicLeft = "bbl";
	public static final String basicMid = "bbm";
	public static final String basicRight = "bbr";

	public static final String chargeLeft = "bcl";
	public static final String chargeRight = "bcr";

	public static final String kAutoroutineDefault = "def";

	public static boolean autoFinished = false;

	// Auto vars
	private static File cmdFile;
	private static FileInputStream fReader;
	private static ObjectInputStream cmdRead;
	private static FileOutputStream fWriter;
	private static ObjectOutputStream cmdWrite;

	static Inputs inputs = new Inputs(2);

	// PathPlanner

	private static PathPlannerTrajectory selectedPath;
	public static Command autoCommand;
	public static HashMap<String, Command> eventMap = new HashMap<>();
	private static FollowPathWithEvents events;

	private static final double maxVel = 4;
	private static final double maxAccel = 1;

	private static PIDController linearController = new PIDController(0.7, 0, 0);
	private static PIDController rotationalController = new PIDController(0.02, 0, 0);

	Auto() {}

	public static void selectAuto(boolean captureReplay, String autoSequence, Supplier<Pose2d> input, Consumer<ChassisSpeeds> output) {
		if (!captureReplay) {
			selectedPath = PathPlanner.loadPath(autoSequence, new PathConstraints(maxVel, maxAccel));
			autoCommand = new PPSwerveControllerCommand(selectedPath, input, linearController, linearController, rotationalController, output);

			events = new FollowPathWithEvents(
				autoCommand,
				selectedPath.getMarkers(),
				eventMap
			);
		} else {
			String filePath = basePath + autoSequence + ".aut";
			System.out.println("Reading auto instructions from " + filePath);
			cmdFile = new File(filePath);
			try {
			fReader = new FileInputStream(cmdFile);
			cmdRead = new ObjectInputStream(fReader);
			} catch(IOException err) {
			System.out.println("Error opening auto file for read: " + err.toString());
			}
		}
	}

	public static Inputs readFile() {
		// System.out.println("Reading auto file...");
		inputs.nullControls();
		try {
		  inputs.ControllerInputs = (ControllerInputs[]) cmdRead.readObject();
		} catch (IOException err) {
		  System.out.println("Finished reading auto file");
		  autoFinished = true;
		} catch (ClassNotFoundException cerr) {
		  System.out.println("Could not read controller inputs object from auto file: " + cerr.toString());
		}
		return inputs;
	}

	// Done in Test part of Robot
	public static void setupRecording(String inputFileName) {
		String filePath = basePath + inputFileName + ".aut";
		System.out.println("Reading auto instructions from " + filePath);
		cmdFile = new File(filePath);
		try {
			fWriter = new FileOutputStream(cmdFile);
			cmdWrite = new ObjectOutputStream(fWriter);
		} catch(IOException err) {
			System.out.println("Error opening auto file for write: " + err.toString());
		}
	}

  	public static void record(Inputs inputs) {
		System.out.println("Writing auto file...");
		try {
			cmdWrite.writeObject(inputs.ControllerInputs);
		} catch(IOException err) {
			System.out.println("Error writing auto file: " + err.toString());
		}
	}

  	public static void closeFile() {
		System.out.println("Auto file closed.");
		if (fReader != null) {
	  		try {
				fReader.close();      
	  		} catch (IOException err) {
				System.out.println("Error closing auto file: " + err.toString());
	  		}
		}
		if (fWriter != null) {
	  		try {
				fWriter.close();
	  		} catch(IOException err) {
				System.out.println("Error closing auto file: " + err.toString());
	  		}
		}
  	}

}