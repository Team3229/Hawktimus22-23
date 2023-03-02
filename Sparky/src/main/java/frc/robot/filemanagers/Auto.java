// Author: Tony Simone (3229 Mentor)

// Auto recording class
//
// NOTE: Verified that we can write multiple serialized objects to a file and
//       deserialize them properly. For the ControllerInputs class, this is
//       fairly compact, also.
//

package frc.robot.filemanagers;

import java.io.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Controller;
import frc.robot.ControllerInputs;
import frc.robot.Dashboard;

public class Auto {
	// Constants
	static final boolean m_recordMode = true; // use this to force disable recording, useful at competitions
	static final boolean WRITE = true;
	static final boolean READ = false;
	static final String basePath = "/home/lvuser/autoFiles/";

	public final SendableChooser <String> heightDropdown = new SendableChooser <> ();
	public final SendableChooser <String> startPosDropdown = new SendableChooser <> ();
	public final SendableChooser <String> grabDropdown = new SendableChooser <> ();

	// height
	public static final String heightHigh = "high";
	public static final String heightMid = "mid";
	public static final String heightLow = "low";

	// 

	// grab
	public static final String grabTaxi = "taxi";
	public static final String grab1 = "grab1";
	public static final String grab2 = "grab2";
	public static final String grab3 = "grab3";
	public static final String grab4 = "grab4";

	public static final String noSelection = "N/A";
	
	public boolean autoFinished = false;

	// Auto vars
	private File cmdFile;
	private FileInputStream fReader;
	private ObjectInputStream cmdRead;
	private FileOutputStream fWriter;
	private ObjectOutputStream cmdWrite;

	private String[] inputFileNames = {null,null,null,null};
	private int autoStep = 1;
	Controller controller = new Controller();
	Dashboard dash = new Dashboard();

	public Auto() {
	}

	public void setupDropdowns() {
		heightDropdown.setDefaultOption("High", heightHigh);
		heightDropdown.addOption("Mid", heightMid);
		heightDropdown.addOption("Low", heightLow);
		heightDropdown.addOption("NO SELECTION", "N/A");

		startPosDropdown.setDefaultOption("NO SELECTION", "N/A");
		startPosDropdown.addOption("Dummy option one", "dC1");

		grabDropdown.setDefaultOption("Only Taxi", grabTaxi);
		grabDropdown.addOption("Grab 1", grab1);
		grabDropdown.addOption("Grab 2", grab2);
		grabDropdown.addOption("Grab 3", grab3);
		grabDropdown.addOption("Grab 4", grab4);
		grabDropdown.addOption("NO SELECTION", "N/A");
		
		dash.putData("Height", heightDropdown);
		dash.putData("Start Position", startPosDropdown);
		dash.putData("Grabbing", grabDropdown);
		dash.putBool("Charge Station", false);

	}

	// Done in Auto part of Robot
	public void setupPlayback(String[] names) {
		inputFileNames = names;
		switch(autoStep) {
			case 1:
				cmdFile = new File(basePath + inputFileNames[1] + inputFileNames[0] + ".aut");
				break;
			case 2:
				cmdFile = new File(basePath + inputFileNames[1] + inputFileNames[2] + ".aut");
				break;
			case 3:
				cmdFile = new File(basePath + inputFileNames[2] + inputFileNames[3] + ".aut");
			case 4:
				// we just finished 3, meaning we're done and should end.
				autoFinished = true;
				return;
        }

		try {
			fReader = new FileInputStream(cmdFile);
			cmdRead = new ObjectInputStream(fReader);
		} catch(IOException err) {
			System.out.println("Error opening auto file for read: " + err.toString());
		}
		// by the time it gets here the first file for placing the starting object is chosen, set, and prepared.
	}

	public ControllerInputs readFile() {
		// System.out.println("Reading auto file...");
		ControllerInputs inputs = controller.nullControls();
		try {
			inputs = (ControllerInputs) cmdRead.readObject();
		} catch (IOException err) {
			// if were finished, check what stage we were in and see if theres another file we need to setup and begin playback for
			closeFile();
			++autoStep;
			setupPlayback(inputFileNames);
		} catch (ClassNotFoundException cerr) {
			System.out.println("Could not read controller inputs object from auto file: " + cerr.toString());
		}
		return inputs;
	}

	// Done in Test part of Robot
	public void setupRecording(String[] inputFileName) {

		if (inputFileName[1] == "N/A") {
			cmdFile = new File(basePath + inputFileNames[2] + inputFileNames[3] + ".aut");
		} else if (inputFileName[2] == "N/A"){
			cmdFile = new File(basePath + inputFileNames[1] + inputFileNames[0] + ".aut");
		} else {
			cmdFile = new File(basePath + inputFileNames[1] + inputFileNames[2] + ".aut");
		}
		
		try {
			if (!cmdFile.exists()) {
				cmdFile.createNewFile();
			}
			fWriter = new FileOutputStream(cmdFile);
			cmdWrite = new ObjectOutputStream(fWriter);
			System.out.println("it is nbeing run");
		} catch(IOException err) {
			System.out.println("Error opening auto file for write: " + err.toString());
		}
	}

	public void record(ControllerInputs inputs) {
		System.out.println("Writing auto file...");
		try {
			cmdWrite.writeObject(inputs);
		} catch(IOException err) {
			System.out.println("Error writing auto file: " + err.toString());
		}
	}

	public void closeFile() {
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