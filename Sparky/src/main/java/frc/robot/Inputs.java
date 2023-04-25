// Author: Tony Simone (3229 Mentor)

// Controller class

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class Inputs {
  //Limits
  static final double STICK_DEADBAND = 0.125;
  static final double TRIGGER_DEADBAND = 0.1;

  // Controller
  public XboxController[] Controllers = {null,null,null,null,null};
  public GenericHID[] Rumbles = {null,null,null,null,null};
  public int controllerCount = 0;
  public ControllerInputs[] ControllerInputs = {null,null,null,null,null};

  public Inputs(int count) {
    controllerCount = count;
    for(int i = 0; i < count; i++){
      Controllers[i] = new XboxController(i);
      Rumbles[i] = new GenericHID(i);
      ControllerInputs[i] = new ControllerInputs();
    }
  }

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