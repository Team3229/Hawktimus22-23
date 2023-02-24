// Author: Tony Simone (3229 Mentor)

// Controller class

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import java.io.Serializable;

class Controller {
  //Limits
  static final double STICK_DEADBAND = 0.2;
  static final double TRIGGER_DEADBAND = 0.1;

  // Controller
  private XboxController d_controller;
  private XboxController m_controller;
  GenericHID m_rumble;
  GenericHID d_rumble;

  Controller() {
    d_controller = new XboxController(0);
    m_controller = new XboxController(1);
    d_rumble = new GenericHID(0);
    m_rumble = new GenericHID(1);
  }

  ControllerInputs getControls() {
    ControllerInputs ci = new ControllerInputs();
    ci.d_rightY = ((Math.abs(d_controller.getRightY()) < STICK_DEADBAND) ? 0 : d_controller.getRightY());
    ci.d_rightX = ((Math.abs(d_controller.getRightX()) < STICK_DEADBAND) ? 0 : d_controller.getRightX());
    ci.d_leftY = ((Math.abs(d_controller.getLeftY()) < STICK_DEADBAND) ? 0 : d_controller.getLeftY());
    ci.d_leftX = ((Math.abs(d_controller.getLeftX()) < STICK_DEADBAND) ? 0 : d_controller.getLeftX());
    ci.d_AButton = d_controller.getAButton();
    ci.d_BButton = d_controller.getBButton();
    ci.d_XButton = d_controller.getXButton();
    ci.d_YButton = d_controller.getYButton();
    ci.d_StartButton = d_controller.getStartButton();
    ci.d_RightBumper = d_controller.getRightBumper();
    ci.d_LeftBumper = d_controller.getLeftBumper();
    ci.d_RightTriggerAxis = ((Math.abs(d_controller.getRightTriggerAxis()) < TRIGGER_DEADBAND) ? 0 : d_controller.getRightTriggerAxis());
    ci.d_LeftTriggerAxis = ((Math.abs(d_controller.getLeftTriggerAxis()) < TRIGGER_DEADBAND) ? 0 : d_controller.getLeftTriggerAxis());
    ci.d_POV = d_controller.getPOV();

    ci.m_rightY = ((Math.abs(m_controller.getRightY()) < STICK_DEADBAND) ? 0 : m_controller.getRightY());
    ci.m_rightX = ((Math.abs(m_controller.getRightX()) < STICK_DEADBAND) ? 0 : m_controller.getRightX());
    ci.m_leftY = ((Math.abs(m_controller.getLeftY()) < STICK_DEADBAND) ? 0 : m_controller.getLeftY());
    ci.m_leftX = ((Math.abs(m_controller.getLeftX()) < STICK_DEADBAND) ? 0 : m_controller.getLeftX());
    ci.m_AButton = m_controller.getAButton();
    ci.m_BButton = m_controller.getBButton();
    ci.m_XButton = m_controller.getXButton();
    ci.m_YButton = m_controller.getYButton();
    ci.m_StartButton = m_controller.getStartButton();
    ci.m_RightBumper = m_controller.getRightBumper();
    ci.m_LeftBumper = m_controller.getLeftBumper();
    ci.m_RightTriggerAxis = ((Math.abs(m_controller.getRightTriggerAxis()) < TRIGGER_DEADBAND) ? 0 : m_controller.getRightTriggerAxis());
    ci.m_LeftTriggerAxis = ((Math.abs(m_controller.getLeftTriggerAxis()) < TRIGGER_DEADBAND) ? 0 : m_controller.getLeftTriggerAxis());
    ci.m_POV = m_controller.getPOV();
    
    return ci;
  }

  ControllerInputs nullControls() {
    ControllerInputs ci = new ControllerInputs();
    ci.d_rightY = 0;
    ci.d_rightX = 0;
    ci.d_leftY = 0;
    ci.d_leftX = 0;
    ci.d_AButton = false;
    ci.d_BButton = false;
    ci.d_XButton = false;
    ci.d_YButton = false;
    ci.d_StartButton = false;
    ci.d_RightBumper = false;
    ci.d_LeftBumper = false;
    ci.d_RightTriggerAxis = 0;
    ci.d_LeftTriggerAxis = 0;
    ci.d_POV = -1;

    ci.m_rightY = 0;
    ci.m_rightX = 0;
    ci.m_leftY = 0;
    ci.m_leftX = 0;
    ci.m_AButton = false;
    ci.m_BButton = false;
    ci.m_XButton = false;
    ci.m_YButton = false;
    ci.m_StartButton = false;
    ci.m_RightBumper = false;
    ci.m_LeftBumper = false;
    ci.m_RightTriggerAxis = 0;
    ci.m_LeftTriggerAxis = 0;
    ci.m_POV = -1;
    return ci;
  }
}

class ControllerInputs implements Serializable {
  // Driver Controls
  double d_rightY;
  double d_rightX;
  double d_leftX;
  double d_leftY;
  boolean d_AButton;
  boolean d_BButton;
  boolean d_XButton;
  boolean d_YButton;
  boolean d_RightBumper;
  boolean d_LeftBumper;
  double d_RightTriggerAxis;
  double d_LeftTriggerAxis;
  int d_POV;
  boolean d_StartButton;

  // Manip Controls
  double m_rightY;
  double m_rightX;
  double m_leftX;
  double m_leftY;
  boolean m_AButton;
  boolean m_BButton;
  boolean m_XButton;
  boolean m_YButton;
  boolean m_RightBumper;
  boolean m_LeftBumper;
  double m_RightTriggerAxis;
  double m_LeftTriggerAxis;
  int m_POV;
  boolean m_StartButton;
}