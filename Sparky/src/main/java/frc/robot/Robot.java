// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Otters: 3229 Programming Sub-Team

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.drivetrain.SwerveKinematics;
import frc.robot.filemanagers.SwerveOffsets;
import frc.robot.filemanagers.Auto;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private String[] selectedAutoSequence = {"","","",""};
    private double[] swerveEncoderVals = {0,0,0,0};
    private boolean autoLeveling = false;

    boolean holdArmPos = false;
    Controller controller = new Controller();
    ControllerInputs inputs;
    double[] dp = {0,0,0};

    SwerveKinematics chassis = new SwerveKinematics();
    Auto auto = new Auto();
    Utils utils = new Utils();
    Leveling leveling = new Leveling();
    Limelight limelight = new Limelight();
    Arm arm = new Arm();
    SwerveOffsets swerveOffsets = new SwerveOffsets();
    Dashboard dash = new Dashboard();

    boolean hasMovedArmManuallyYet = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        chassis.navxGyro.zeroYaw();
        chassis.navxGyro.calibrate();

        auto.closeFile();
        auto.setupDropdowns();

        chassis.configPIDS();

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

        swerveEncoderVals = chassis.encoderValues();
        // TODO
        dash.putNumber("frontLeft", swerveEncoderVals[0]);
        dash.putNumber("frontRight", swerveEncoderVals[1]);
        dash.putNumber("backLeft", swerveEncoderVals[2]);
        dash.putNumber("backRight", swerveEncoderVals[3]);

        // TODO
        dash.putNumber("intakeAngle", arm.getIntakeEncoder());
        dash.putNumber("armAngle", arm.getArmEncoder());

        // TODO
        dash.putNumber("CAN Uilization", Math.floor(RobotController.getCANStatus().percentBusUtilization*100));

        arm.pcm.enableCompressorDigital();
        limelight.getValues();

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

        inputs = controller.nullControls();

        auto.closeFile();
        chassis.configPIDS();

        selectedAutoSequence[0] = auto.heightDropdown.getSelected();
        selectedAutoSequence[1] = auto.startPosDropdown.getSelected();
        selectedAutoSequence[2] = auto.grabDropdown.getSelected();
        selectedAutoSequence[3] = String.valueOf(dash.readBool("Charge Station"));
        System.out.println("Auto selected: " + selectedAutoSequence);
        auto.setupPlayback(selectedAutoSequence);

        autoLeveling = false;

        auto.autoFinished = false;
        auto.autoStep = 1;
        holdArmPos = false;

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        if (auto.autoFinished) {
            chassis.stop();
            inputs = controller.nullControls();
        } else {
            inputs = auto.readFile();
            RunControls();
        }

    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {

        inputs = controller.nullControls();

        auto.closeFile();

        chassis.configPIDS();

        autoLeveling = false;
        holdArmPos = false;

        if (dash.readBool("resetAngleOffsets")) {
            chassis.fixOffsets();
            System.out.println("Reset Offsets");
            dash.putBool("resetAngleOffsets", false);
        }

        arm.goalLevel = 0;

        arm.pcm.enableCompressorDigital();

        hasMovedArmManuallyYet = false;

    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        inputs = controller.getControls();
        RunControls();

    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        controller.d_rumble.setRumble(RumbleType.kBothRumble, 0);

    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {

        inputs = controller.nullControls();

        // record a new auto sequence
        selectedAutoSequence[0] = auto.heightDropdown.getSelected();
        selectedAutoSequence[1] = auto.startPosDropdown.getSelected();
        selectedAutoSequence[2] = auto.grabDropdown.getSelected();
        selectedAutoSequence[3] = String.valueOf(dash.readBool("Charge Station"));

        auto.setupRecording(selectedAutoSequence);

        autoLeveling = false;
        holdArmPos = false;

    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

        inputs = controller.getControls();
        auto.record(inputs);
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

    void ExecuteDriveControls() {

        //Warn driver they are drawing too much power out of battery (usually because driving too fast)
        if (RobotController.getBatteryVoltage() < 9) {
            controller.d_rumble.setRumble(RumbleType.kBothRumble, 0.3);
        } else {
            controller.d_rumble.setRumble(RumbleType.kBothRumble, 0);
        }

        // Drive swerve chassis with joystick deadbands
        if (!DriverStation.isJoystickConnected(0)) {
            chassis.stop();
        } else {
            if (Math.abs(inputs.d_leftX) > 0 | Math.abs(inputs.d_leftY) > 0) {
                chassis.drive(inputs.d_leftX, inputs.d_leftY, inputs.d_rightX);
            } else {
                // D-Pad slow driving

                if (inputs.d_POV != -1) {
                    dp = utils.getDirectionalPadValues(inputs.d_POV);
                    chassis.drive(dp[0] / 3, dp[1] / 3, inputs.d_rightX);
                } else if (Math.abs(inputs.d_rightX) > 0){
                    chassis.drive(0, 0, inputs.d_rightX);
                } else {
                    chassis.stop();
                }
            }
        }


        // toggle auto level (only for autonomous)
        if (inputs.d_YButton && DriverStation.isAutonomousEnabled()) {
            autoLeveling = true;
        }

        // if we're auto leveling, move to work
        if (autoLeveling) {
            chassis.drive(0, Leveling.getBalanced(chassis.navxGyro.getRoll()), 0);
        }

        // Reset field orientation
        if (inputs.d_AButton) {
            chassis.navxGyro.zeroYaw();
        }

        // Line up with nearest cube grid
        if (inputs.d_XButton) {
            if (!arm.holdingCone & !arm.holdingCube) {
                double[] speeds = limelight.alignWithTag(arm.holdingCube);
                chassis.drive(speeds[0], speeds[1], speeds[2]);
            }
        }

    }


    //Manip Controls
    void ExecuteManipControls() {
        // dP for controlling arm levels
        switch (inputs.m_POV) {
            case 0:
                // up - High
                hasMovedArmManuallyYet = false;
                arm.goalLevel = 3;
                holdArmPos = false;
                break;
            case 90|270:
                // left/right - Mid
                hasMovedArmManuallyYet = false;
                arm.goalLevel = 2;
                holdArmPos = false;
                break;
            case 180:
                // down - Hybrid
                hasMovedArmManuallyYet = false;
                arm.goalLevel = 1;
                holdArmPos = false;
                break;
    
        }

        if (inputs.m_BackButton) {
            // back - Dock
            hasMovedArmManuallyYet = false;
            arm.goalLevel = 4;
            holdArmPos = false;
        }
        
        // if the manip controller isn't connected, don't run the arm (cause safety)
        if(!DriverStation.isJoystickConnected(1)){
            arm.stopArm(true, true);
        } else if (inputs.m_POV == -1 & inputs.m_leftY == 0 & inputs.m_rightY == 0){
            // if we have no input, stop the motors.
            arm.stopArm(true, true);
            if(!holdArmPos && arm.goalLevel == 0){
                // if we have no input, not already holding, and are not using dp to go somewhere, start holding arm up
                arm.holdAng[0] = arm.getArmEncoder();
                arm.holdAng[1] = arm.getIntakeEncoder();
                holdArmPos = true;
            }
        } else if(inputs.m_leftY != 0 | inputs.m_rightY != 0){
            // if we have a controller, have input on the sticks, we are manually moving. This should automatically override dp movement.
            arm.goalLevel = 0;
            // manual arming
            // moving, no hold.
            // if we are using sticks we need to stop the dp movement, manual should ovverride it.
            holdArmPos = false;
            
            // tolerances so we don't kill the intake:
            if ((arm.getIntakeEncoder() <= 78 & inputs.m_rightY < 0) | (arm.getIntakeEncoder() >= 330 & inputs.m_rightY > 0)) {
                arm.stopArm(false, true);
            } else {
                arm.intakeArmMotor.set(inputs.m_rightY*0.3);
            }

            // tolerances so we don't kill the arm:
            if ((arm.getArmEncoder() >= 320 & arm.getArmEncoder() < 350 & inputs.m_leftY > 0) | (arm.getArmEncoder() <= 5 & inputs.m_leftY < 0)) {
                arm.stopArm(true, false);
            } else {
                arm.armMotor.set(inputs.m_leftY*0.2);
            }
        }

        // grabbing cube
        if (inputs.m_LeftBumper) {
            arm.grabObject(true);
        } else if (inputs.m_RightBumper) {
            arm.placeObject(false);
        } else {
            arm.leftWheels.stopMotor();
            arm.rightWheels.stopMotor();
        }

        // Grabbing cone
        if (inputs.m_LeftTriggerAxis > 0.1) {
            arm.placeObject(true);
        } else {
            if (inputs.m_RightTriggerAxis > 0.1) {
                arm.grabObject(false);
            }
        }

        // update arm
        arm.runArm(holdArmPos);

    }
}