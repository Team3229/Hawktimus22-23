// Otter: Tony Simone (3229 Mentor)

// Auto recording class
//
// NOTE: Verified that we can write multiple serialized objects to a file and
//       deserialize them properly. For the ControllerInputs class, this is
//       fairly compact, also.
//

package frc.robot;

import java.io.*;

class Auto {
  // Constants
  static final boolean m_recordMode = true; // use this to force disable recording, useful at competitions
  static final boolean WRITE = true;
  static final boolean READ = false;
  static final String basePath = "/home/lvuser/";
  static final String kTestAuto = "doSomething";
  static final String kAutoroutineDefault = kTestAuto;

  boolean autoFinished = false;

  // Auto vars
  private File cmdFile;
  private FileInputStream fReader;
  private ObjectInputStream cmdRead;
  private FileOutputStream fWriter;
  private ObjectOutputStream cmdWrite;

  Controller controller = new Controller();


  Auto() {
  }

  // Done in Auto part of Robot
  void SetupPlayback(String inputFileName) {
    String filePath = basePath + inputFileName + ".aut";
    System.out.println("Reading auto instructions from " + filePath);
    cmdFile = new File(filePath);
    try {
      fReader = new FileInputStream(cmdFile);
      cmdRead = new ObjectInputStream(fReader);
    } catch(IOException err) {
      System.out.println("Error opening auto file for read: " + err.toString());
    }
  }

  ControllerInputs readFile() {
    // System.out.println("Reading auto file...");
    ControllerInputs inputs = controller.nullControls();
    try {
      inputs = (ControllerInputs) cmdRead.readObject();
    } catch (IOException err) {
      System.out.println("Finished reading auto file");
      autoFinished = true;
    } catch (ClassNotFoundException cerr) {
      System.out.println("Could not read controller inputs object from auto file: " + cerr.toString());
    }
    return inputs;
  }

  // Done in Test part of Robot
  void setupRecording(String inputFileName) {
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

  void Record(ControllerInputs inputs) {
    System.out.println("Writing auto file...");
    try {
      cmdWrite.writeObject(inputs);
    } catch(IOException err) {
      System.out.println("Error writing auto file: " + err.toString());
    }
  }

  void CloseFile() {
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