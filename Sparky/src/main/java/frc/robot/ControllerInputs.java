package frc.robot;

import java.io.Serializable;

public class ControllerInputs implements Serializable {
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
    boolean d_BackButton;
  
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
    boolean m_AButtonPressed;
    double m_RightTriggerAxis;
    double m_LeftTriggerAxis;
    int m_POV;
    boolean m_StartButton;
    boolean m_BackButton;
  }