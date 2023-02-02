// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private String selectedAuto;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private double[] encVals = {0, 0, 0, 0};
  private boolean autoLevel = false;

  Controller controller = new Controller();
  ControllerInputs inputs;
  double[] dp = {0,0,0};
  double[] limelightDist = {0,0};

  SwerveKinematics chassis = new SwerveKinematics();
  Auto auto = new Auto();
  Utils utils = new Utils();
  Leveling leveling = new Leveling();
  Limelight limelight = new Limelight();
  Arm arm = new Arm();

  double bufferZone = 0.1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("doSomething", Auto.kTestAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    chassis.navxGyro.zeroYaw();
    chassis.navxGyro.calibrate();

    SmartDashboard.putNumber("driveP", chassis.drivePID[0]);
    SmartDashboard.putNumber("driveI", chassis.drivePID[1]);
    SmartDashboard.putNumber("driveD", chassis.drivePID[2]);

    SmartDashboard.putNumber("angleP", chassis.anglePID[0]);
    SmartDashboard.putNumber("angleI", chassis.anglePID[1]);
    SmartDashboard.putNumber("angleD", chassis.anglePID[2]);

    SmartDashboard.putNumber("navxGs", 0);

    auto.CloseFile();

    chassis.ConfigPIDS();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    encVals = chassis.EncoderValues();
    SmartDashboard.putNumber("frontLeft", encVals[0]);
    SmartDashboard.putNumber("frontRight", encVals[1]);
    SmartDashboard.putNumber("backLeft", encVals[2]);
    SmartDashboard.putNumber("backRight", encVals[3]);
    SmartDashboard.putNumber("navXGyro", chassis.robotRotation);

    SmartDashboard.putNumber("navxGs", chassis.navxGyro.getAccelFullScaleRangeG());

    limelight.GetValues();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    auto.CloseFile();
    chassis.ConfigPIDS();

    selectedAuto = m_chooser.getSelected();
    System.out.println("Auto selected: " + selectedAuto);
    auto.SetupPlayback(selectedAuto);

    inputs = controller.nullControls();

    autoLevel = false;

    auto.autoFinished = false;

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    if (auto.autoFinished) {
      chassis.Stop();
      inputs = controller.nullControls();
    } else {
      inputs = auto.ReadFile();
      RunControls();
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    auto.CloseFile();

    chassis.drivePID[0] = SmartDashboard.getNumber("driveP", 0);
    chassis.drivePID[1] = SmartDashboard.getNumber("driveI", 0);
    chassis.drivePID[2] = SmartDashboard.getNumber("driveD", 0);

    chassis.anglePID[0] = SmartDashboard.getNumber("angleP", 0);
    chassis.anglePID[1] = SmartDashboard.getNumber("angleI", 0);
    chassis.anglePID[2] = SmartDashboard.getNumber("angleD", 0);

    chassis.ConfigPIDS();

    inputs = controller.nullControls();

    autoLevel = false;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    inputs = controller.getControls();
    RunControls();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {

    inputs = controller.nullControls();
    auto.SetupRecording(m_chooser.getSelected());

    autoLevel = false;

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    inputs = controller.getControls();
    auto.Record(inputs);
    RunControls();

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

void RunControls() {
  ExecuteDriveControls();
  ExecuteManipControls();
}

void ExecuteDriveControls(){

  // Drive swerve chassis with joystick deadbands
  if (Math.abs(inputs.d_leftX) > 0 | Math.abs(inputs.d_leftY) > 0 | Math.abs(inputs.d_rightX) > 0) {
      chassis.Drive(inputs.d_leftX, inputs.d_leftY, inputs.d_rightX);
  } else {
    // D-Pad driving slowly
    dp = utils.dirPad(inputs.d_POV);
    chassis.Drive(dp[0]/2,dp[1]/2,dp[2]/2);
  }

  // Zero NavX Gyro
  if(inputs.d_AButton){
    chassis.navxGyro.zeroYaw();
  }

  // toggle auto level
  if(inputs.d_YButton){
    autoLevel = true;
  }

  // if we're auto leveling, move to work
  if (autoLevel) {
    chassis.Drive(0, Leveling.GetBalanced(chassis.navxGyro.getRoll()), 0);
  }

  arm.checkHandMotors();
    
}
void ExecuteManipControls(){
  if(arm.compressor.getPressure() >= 50){
    arm.compressor.disable();
  }
  // grabbing things functions
  if (inputs.m_LeftBumper) {
    arm.startCubeGrab();
  } else if (inputs.m_RightBumper){
    arm.placeObject(true);
  } else {
    arm.softStop();
  }

  if(inputs.m_AButton){
    // double[] a = limelight.alignWithTag();
    // chassis.Drive(a[0], a[1], a[2]);
    
  }
  if(inputs.m_BButton){
    // close solenoid
    arm.offSolenoid.set(true);
    arm.onSolenoid.set(false);
  }
  if(inputs.m_XButton){
    // open/on
    arm.offSolenoid.set(false);
    arm.onSolenoid.set(true);
  }
  if(inputs.m_YButton){
    // shut it all down.
    arm.offSolenoid.set(false);
    arm.onSolenoid.set(false);
  }
}
}