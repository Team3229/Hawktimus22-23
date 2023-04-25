// Author: Tony Simone (3229 Mentor)

// Controller class

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
/**
 * Inputs class that handles all controller inputs, works for Xbox controllers only.
 */
public class Inputs {
  /** Deadband for the sticks */
  static final double STICK_DEADBAND = 0.125;
  /** Deadband for the triggers */
  static final double TRIGGER_DEADBAND = 0.1;

  /** Array of controllers */
  public XboxController[] Controllers = {null,null,null,null,null};
  /** Array of rumbles for each controller, used for vibrating controllers */
  public GenericHID[] Rumbles = {null,null,null,null,null};
  /** The count of controllers connected, DO NOT CHANGE */
  public int controllerCount = 0;
  /** Array of the inputs for each of the connected controllers */
  public ControllerInputs[] ControllerInputs = {null,null,null,null,null};

  /**
   * Constructor for the Inputs class. Initilizes the controllerCount, Controllers, Rumbles, and ControllerInputs items.
   * @param count (Int) The number of controllers, Max 5; Min 1
   */
  public Inputs(int count) {
    controllerCount = count;
    for(int i = 0; i < count; i++){
      Controllers[i] = new XboxController(i);
      Rumbles[i] = new GenericHID(i);
      ControllerInputs[i] = new ControllerInputs();
    }
  }

  /**
   * Gets the controllers from the controllers for each connected controller, saves results to ControllerInputs array.
   */
  void getControls() {
    //Gets controls for each assigned controller
    for(var i = 0; i < controllerCount; i++){
      ControllerInputs[i].rightY = ((Math.abs(Controllers[i].getRightY()) < STICK_DEADBAND) ? 0 : Controllers[i].getRightY());
      ControllerInputs[i].rightX = ((Math.abs(Controllers[i].getRightX()) < STICK_DEADBAND) ? 0 : Controllers[i].getRightX());
      ControllerInputs[i].leftY = ((Math.abs(Controllers[i].getLeftY()) < STICK_DEADBAND) ? 0 : Controllers[i].getLeftY());
      ControllerInputs[i].leftX = ((Math.abs(Controllers[i].getLeftX()) < STICK_DEADBAND) ? 0 : Controllers[i].getLeftX());
      ControllerInputs[i].AButton = Controllers[i].getAButton();
      ControllerInputs[i].BButton = Controllers[i].getBButton();
      ControllerInputs[i].XButton = Controllers[i].getXButton();
      ControllerInputs[i].YButton = Controllers[i].getYButton();
      ControllerInputs[i].StartButton = Controllers[i].getStartButton();
      ControllerInputs[i].BackButton = Controllers[i].getBackButton();
      ControllerInputs[i].RightBumper = Controllers[i].getRightBumper();
      ControllerInputs[i].LeftBumper = Controllers[i].getLeftBumper();
      ControllerInputs[i].RightTriggerAxis = ((Math.abs(Controllers[i].getRightTriggerAxis()) < TRIGGER_DEADBAND) ? 0 : Controllers[i].getRightTriggerAxis());
      ControllerInputs[i].LeftTriggerAxis = ((Math.abs(Controllers[i].getLeftTriggerAxis()) < TRIGGER_DEADBAND) ? 0 : Controllers[i].getLeftTriggerAxis());
      ControllerInputs[i].POV = Controllers[i].getPOV();
      ControllerInputs[i].AButtonPressed = Controllers[i].getAButtonPressed();
      ControllerInputs[i].AButtonPressed = Controllers[i].getAButtonReleased();
      ControllerInputs[i].BButtonPressed = Controllers[i].getBButtonPressed();
      ControllerInputs[i].BButtonReleased = Controllers[i].getBButtonReleased();
      }
  }

  /**
   * Nulls the controlls in the ControllerInputs array
   */
  public void nullControls() {
    for(var i = 0; i < controllerCount; i++){
      ControllerInputs[i].rightY = 0;
      ControllerInputs[i].rightX = 0;
      ControllerInputs[i].leftY = 0;
      ControllerInputs[i].leftX = 0;
      ControllerInputs[i].AButton = false;
      ControllerInputs[i].BButton = false;
      ControllerInputs[i].XButton = false;
      ControllerInputs[i].YButton = false;
      ControllerInputs[i].StartButton = false;
      ControllerInputs[i].BackButton = false;
      ControllerInputs[i].RightBumper = false;
      ControllerInputs[i].LeftBumper = false;
      ControllerInputs[i].RightTriggerAxis = 0;
      ControllerInputs[i].LeftTriggerAxis = 0;
      ControllerInputs[i].POV = -1;
    }
  }
}