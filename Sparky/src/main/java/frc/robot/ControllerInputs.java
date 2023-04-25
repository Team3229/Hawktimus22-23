package frc.robot;

import java.io.Serializable;

/**
 * Controller Inputs class, implemented in Inputs.
 * @see Inputs
 */
public class ControllerInputs implements Serializable {
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
    boolean BackButton;
    boolean AButtonPressed;
    boolean AButtonReleased;
    boolean BButtonPressed;
    boolean BButtonReleased;
  }