// Otter: Tony Simone (3229 Mentor)

// Controller class

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import java.io.Serializable;

class Controller {
  //Limits
  static final double stickDeadband = 0.2;
  static final double triggerDeadband = 0.1;

  // Controller
  private XboxController controller;

  Controller() {
    controller = new XboxController(0);
  }

  ControllerInputs getControls() {
    ControllerInputs ci = new ControllerInputs();
    ci.rightY = ((Math.abs(controller.getRightY()) < stickDeadband) ? 0 : controller.getRightY());
    ci.rightX = ((Math.abs(controller.getRightX()) < stickDeadband) ? 0 : controller.getRightX());
    ci.leftY = ((Math.abs(controller.getLeftY()) < stickDeadband) ? 0 : controller.getLeftY());
    ci.leftX = ((Math.abs(controller.getLeftX()) < stickDeadband) ? 0 : controller.getLeftX());
    ci.AButton = controller.getAButton();
    ci.BButton = controller.getBButton();
    ci.XButton = controller.getXButton();
    ci.YButton = controller.getYButton();
    ci.StartButton = controller.getStartButton();
    ci.RightBumper = controller.getRightBumper();
    ci.LeftBumper = controller.getLeftBumper();
    ci.RightTriggerAxis = ((Math.abs(controller.getRightTriggerAxis()) < triggerDeadband) ? 0 : controller.getRightTriggerAxis());
    ci.LeftTriggerAxis = ((Math.abs(controller.getLeftTriggerAxis()) < triggerDeadband) ? 0 : controller.getLeftTriggerAxis());
    ci.POV = controller.getPOV();
    return ci;
  }

  ControllerInputs nullControls() {
    ControllerInputs ci = new ControllerInputs();
    ci.rightY = 0;
    ci.rightX = 0;
    ci.leftY = 0;
    ci.leftX = 0;
    ci.AButton = false;
    ci.BButton = false;
    ci.XButton = false;
    ci.YButton = false;
    ci.StartButton = false;
    ci.RightBumper = false;
    ci.LeftBumper = false;
    ci.RightTriggerAxis = 0;
    ci.LeftTriggerAxis = 0;
    ci.POV = -1;
    return ci;
  }
}

class ControllerInputs  implements Serializable {
  // Driver Controls
  double rightY;
  double rightX;
  double leftX;
  double leftY;
  boolean AButton;
  boolean BButton;
  boolean XButton;
  boolean YButton;
  boolean RightBumper;
  boolean LeftBumper;
  double RightTriggerAxis;
  double LeftTriggerAxis;
  int POV;
  boolean StartButton;
}