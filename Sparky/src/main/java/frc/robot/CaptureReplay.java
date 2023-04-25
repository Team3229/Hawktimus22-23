// Author: Tony Simone (3229 Mentor)

// Auto recording class
//
// NOTE: Verified that we can write multiple serialized objects to a file and
//       deserialize them properly. For the ControllerInputs class, this is
//       fairly compact, also.
//

package frc.robot;

import java.io.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Capture replay class, class for recording the controller inputs during test mode and playing them back in auto mode. Requires Inputs and ControllerInputs
 * @see Inputs
 * @see ControllerInputs 
 */
public class CaptureReplay {

	static final boolean WRITE = true;
	static final boolean READ = false;
	/** Path on the RIO to save the auto files, make sure a directory is created before using if applicable. */
	static final String basePath = "/home/lvuser/autoFiles/";

	/** Example dropdown for choosing an auto sequence, create multiple with better names as needed. */
	public final SendableChooser <String> DummyDropdown = new SendableChooser <> ();
	/** Example option for a dropdown */
	public static final String DummyOption = "Dummy";

	/** No selection option, add to all dropdowns. */
	public static final String noSelection = "N/A";
	
	/** If Capture replay has finished all autos. */
	public boolean autoFinished = false;

	private File cmdFile;
	private FileInputStream fReader;
	private ObjectInputStream cmdRead;
	private FileOutputStream fWriter;
	private ObjectOutputStream cmdWrite;

	/** Have inputs started during recording */
	public boolean inputsStarted = false;
	/** Number of cached null frames during recording */
	public int cachedNulls = 0;

	/** String array of the chosen input file names */
	private String[] inputFileNames;
	/** The current step in the auto sequence */
	private int autoStep = 1;
	/** The inputs instance */
	Inputs inputs = new Inputs(2);

	public CaptureReplay() {}

	/** Sets up the dropdowns and their options */
	public void setupDropdowns() {
		DummyDropdown.setDefaultOption("Dummy", DummyOption);
		DummyDropdown.addOption("NO SELECTION", "N/A");
		
		SmartDashboard.putData("Dummy", DummyDropdown);
	}

	/**
	 * Sets up capture replay for the playback of an auto sequence
	 * @param names (String[]) An array of the file identifiers, selected from the dropdowns
	 */
	public void setupPlayback(String[] names) {
		inputFileNames = names;
		switch(autoStep) {
			//Add more cases between 1 and 2, incrementing by one for each stage for autos, changing the paths as necessary.
			case 1:
				cmdFile = new File(basePath + inputFileNames[0] + ".aut");
				break;
			case 2:
				//End of auto.
				autoFinished = true;
				return;
        }
		try {
			fReader = new FileInputStream(cmdFile);
			cmdRead = new ObjectInputStream(fReader);
		} catch(IOException err) {
			System.out.println("Error opening auto file for read: " + err.toString());
		}
	}

	/**
	 * Reads the next frame from the selected auto file and handles continuing the sequence if we reach the end of the currently selected file
	 * @return (Inputs) The inputs instance of the next frame
	 */
	public Inputs readFile() {
		// System.out.println("Reading auto file...");
		inputs.nullControls();
		try {
			inputs.ControllerInputs = (ControllerInputs[]) cmdRead.readObject();
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

	/**
	 * Sets up capture replay for recording a new file or rewriting an old one
	 * @param inputFileName (String[]) An array of the file identifiers, selected from the dropdowns
	 */
	public void setupRecording(String[] inputFileName) {
		inputsStarted = false;
		//Example of how to do it with more than one
		// if (inputFileName[1] == "N/A") {
		// 	cmdFile = new File(basePath + inputFileNames[2] + inputFileNames[3] + ".aut");
		// } else if (inputFileName[2] == "N/A"){
		// 	cmdFile = new File(basePath + inputFileNames[1] + inputFileNames[0] + ".aut");
		// } else {
		// 	cmdFile = new File(basePath + inputFileNames[1] + inputFileNames[2] + ".aut");
		// }
		//Init the file we have selected
		cmdFile = new File(basePath + inputFileNames[0] + ".aut");
		
		try {
			fWriter = new FileOutputStream(cmdFile);
			cmdWrite = new ObjectOutputStream(fWriter);
		} catch(IOException err) {
			System.out.println("Error opening auto file for write: " + err.toString());
		}
	}

	/**
	 * Records the current frame of inputs to the selected file, does not record blank frames at the starts and ends of an auto sequence.
	 * @param inputs (Inputs) The inputs for the current frame, to be recorded.
	 */
	public void record(Inputs inputs) {
		System.out.println("Writing auto file...");
		Inputs dummyInputs = inputs;
		dummyInputs.nullControls();
		if (inputsStarted) {
			// Recording input, means there has already been at least a single input this recording. 
			if (inputs.ControllerInputs.equals(dummyInputs.ControllerInputs)) {
				// If there is no input, remember that there was a frame of no inputs.
				++cachedNulls;
			} else if (cachedNulls > 0) {
				// If there is an input, and we have nulls to cache, record them and then record the input we just recieved.
				for (int x = 0; x <= cachedNulls; ++x) {
					try {
						System.out.println("Writing null frame...");
						cmdWrite.writeObject(dummyInputs.ControllerInputs);
					} catch(IOException err) {
						System.out.println("Error writing null frame: " + err.toString());
					}
				}
				//Record the inputs frame that caused this to happen now
				try {
					System.out.println("Writing null frame...");
					cmdWrite.writeObject(inputs.ControllerInputs);
				} catch(IOException err) {
					System.out.println("Error writing null frame: " + err.toString());
				}
				//Reset cached nulls
				cachedNulls = 0;
			} else {
				// Have an input, and no nulls to cache, write a regular frame of inputs.
				try {
					System.out.println("Writing auto file...");
					cmdWrite.writeObject(inputs.ControllerInputs);
				} catch(IOException err) {
					System.out.println("Error writing auto file: " + err.toString());
				}
			}
		} else if (inputs.ControllerInputs.equals(dummyInputs.ControllerInputs)) {
			// Waiting for not null input to start recording
			System.out.println("Waiting for controller input...");
		} else {
			// Start recording input
			//This should only run once, the first frame of actual input that is not null.
			System.out.println("Started recording");
			inputsStarted = true;
			try {
				System.out.println("Writing auto file...");
				cmdWrite.writeObject(inputs.ControllerInputs);
			} catch(IOException err) {
				System.out.println("Error writing auto file: " + err.toString());
			}
		}
	}

	/** Closes the currently selected file */
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