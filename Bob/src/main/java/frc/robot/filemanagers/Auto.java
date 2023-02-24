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
	static final String basePath = "/home/lvuser/";

	public final SendableChooser <String> heightDropdown = new SendableChooser <> ();
	public final SendableChooser <String> startPosDropdown = new SendableChooser <> ();
	public final SendableChooser <String> grabDropdown = new SendableChooser <> ();
	// height
	public static final String heightHigh = "high";
	public static final String heightMid = "mid";
	public static final String heightLow = "low";

	// grab
	public static final String grabTaxi = "taxi";
	public static final String grabCone1 = "cone1";
	public static final String grabCone2 = "cone2";
	public static final String grabCone3 = "cone3";
	public static final String grabCone4 = "cone4";
	public static final String grabCube1 = "cube1";
	public static final String grabCube2 = "cube2";
	public static final String grabCube3 = "cube3";
	public static final String grabCube4 = "cube4";
	
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
		heightDropdown.setDefaultOption("High", Auto.heightHigh);
		heightDropdown.addOption("Mid", Auto.heightMid);
		heightDropdown.addOption("Low", Auto.heightLow);
		startPosDropdown.setDefaultOption("SELECT AN OPTION", "NO SELECTION");
		startPosDropdown.addOption("Dummy option one", "dummyCase1");
		grabDropdown.setDefaultOption("Only Taxi", Auto.grabTaxi);
		grabDropdown.addOption("Grab Cone 1", Auto.grabCone1);
		grabDropdown.addOption("Grab Cone 2", Auto.grabCone2);
		grabDropdown.addOption("Grab Cone 3", Auto.grabCone3);
		grabDropdown.addOption("Grab Cone 4", Auto.grabCone4);
		grabDropdown.addOption("Grab Cube 1", Auto.grabCone1);
		grabDropdown.addOption("Grab Cube 2", Auto.grabCone2);
		grabDropdown.addOption("Grab Cube 3", Auto.grabCone3);
		grabDropdown.addOption("Grab Cube 4", Auto.grabCone4);
		
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
				switch(inputFileNames[1]){
					// Place starting item switch
					case "dummyCase1":
						switch(inputFileNames[0]){
							case "high":
								// High and dummycase1 starting pos, setup the according file.
								String highDummyCasePath = basePath + "hDC1" + ".aut";
								cmdFile = new File(highDummyCasePath);
								break;
							case "mid":
								String midDummyCasePath = basePath + "mDC1" + ".aut";
								cmdFile = new File(midDummyCasePath);
								break;
							case "low":
								String lowDummyCasePath = basePath + "lDC1" + ".aut";
								cmdFile = new File(lowDummyCasePath);
								break;
						}
						break;
				} 
				break;
			case 2:
				switch(inputFileNames[1]) {
					// Grab next item/taxi switch
          // switces over starting pos
				}
				break;
			case 3:
				switch(inputFileNames[3]){
          // charge station switch
          // switch over charge
        }
					
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
			++autoStep;
			setupPlayback(inputFileNames);
		} catch (ClassNotFoundException cerr) {
			System.out.println("Could not read controller inputs object from auto file: " + cerr.toString());
		}
		return inputs;
	}

	// Done in Test part of Robot
	public void setupRecording(String[] inputFileName) {
		switch(inputFileName[1]){
				case "dummyCase1":
					switch(inputFileName[0]){
						case "high":
							// High and dummycase1 starting pos, setup the according file.
							String highDummyCasePath = basePath + "hDC1" + ".aut";
							cmdFile = new File(highDummyCasePath);
							break;
						case "mid":
							String midDummyCasePath = basePath + "mDC1" + ".aut";
							cmdFile = new File(midDummyCasePath);
							break;
						case "low":
							String lowDummyCasePath = basePath + "lDC1" + ".aut";
							cmdFile = new File(lowDummyCasePath);
							break;
					}
					break;
		}
		try {
			fWriter = new FileOutputStream(cmdFile);
			cmdWrite = new ObjectOutputStream(fWriter);
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