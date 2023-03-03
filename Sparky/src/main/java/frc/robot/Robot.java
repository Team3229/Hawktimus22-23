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
    private String[] selectedAuto = {"","","",""};
    private double[] encVals = {0,0,0,0};
    private boolean autoLevel = false;

    boolean hold = false;
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
    SwerveOffsets swerveOffsets = new SwerveOffsets();
    Dashboard dash = new Dashboard();

    double bufferZone = 0.1;
    boolean controllerError = false;
    boolean hasMovedArmManuallyYet = false;
    boolean inAuto = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        auto.setupDropdowns();

        chassis.navxGyro.zeroYaw();
        chassis.navxGyro.calibrate();

        auto.closeFile();

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

        encVals = chassis.encoderValues();
        dash.putNumber("frontLeft", encVals[0]);
        dash.putNumber("frontRight", encVals[1]);
        dash.putNumber("backLeft", encVals[2]);
        dash.putNumber("backRight", encVals[3]);
        arm.pcm.enableCompressorDigital();
        limelight.getValues();

        dash.putNumber("intakeAngle", arm.getIntakeEncoder());
        dash.putNumber("armAngle", arm.getArmEncoder());

        dash.putNumber("CAN Uilization", Math.floor(RobotController.getCANStatus().percentBusUtilization*100));

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

        auto.closeFile();
        chassis.configPIDS();

        selectedAuto[0] = auto.heightDropdown.getSelected();
        selectedAuto[1] = auto.startPosDropdown.getSelected();
        selectedAuto[2] = auto.grabDropdown.getSelected();
        selectedAuto[3] = String.valueOf(dash.readBool("Charge Station"));
        System.out.println("Auto selected: " + selectedAuto);
        auto.setupPlayback(selectedAuto);

        inputs = controller.nullControls();

        autoLevel = false;

        auto.autoFinished = false;
        hold = false;
        inAuto = true;

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

        auto.closeFile();

        chassis.configPIDS();

        inputs = controller.nullControls();

        autoLevel = false;
        hold = false;
        inAuto = false;

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
        // record a new auto sequence
        selectedAuto[0] = auto.heightDropdown.getSelected();
        selectedAuto[1] = auto.startPosDropdown.getSelected();
        selectedAuto[2] = auto.grabDropdown.getSelected();
        selectedAuto[3] = String.valueOf(dash.readBool("Charge Station"));
        inputs = controller.nullControls();
        auto.setupRecording(selectedAuto);

        autoLevel = false;
        hold = false;
        inAuto = true;

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

        //Warn driver they are going too fast on swerve
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
                // D-Pad driving slowly

                if (inputs.d_POV != -1) {
                    dp = utils.getDirectionalPadValues(inputs.d_POV);
                    chassis.drive(dp[0] / 2, dp[1] / 2, inputs.d_rightX);
                } else if (Math.abs(inputs.d_rightX) > 0){
                    chassis.drive(0, 0, inputs.d_rightX);
                } else {
                    chassis.stop();
                }
            }
        }


        // toggle auto level (only for autonomous)
        if (inputs.d_YButton && inAuto) {
            autoLevel = true;
        }

        // if we're auto leveling, move to work
        if (autoLevel) {
            chassis.drive(0, Leveling.getBalanced(chassis.navxGyro.getRoll()), 0);
        }

        // Reset field orientation
        if (inputs.d_AButton) {
            chassis.navxGyro.zeroYaw();
        }

    }


    //Manip Controls
    void ExecuteManipControls() {
        dash.putNumber("m_inputsPOV", inputs.m_POV);
        // dP for controlling arm levels
        switch (inputs.m_POV) {
            case 0:
                // up - High
                arm.setCurrentLevel(3);
                hasMovedArmManuallyYet = false;
                hold = false;
                break;
            case 90 | 270:
                // right - Mid
                hasMovedArmManuallyYet = false;
                arm.setCurrentLevel(2);
                hold = false;
                break;
            case 180:
                // down - Hybrid
                hasMovedArmManuallyYet = false;
                arm.setCurrentLevel(1);
                hold = false;
                break;
    
        }
        
        if(!DriverStation.isJoystickConnected(1)){
            arm.armMotor.stopMotor();
            arm.intakeArmMotor.stopMotor();
        } else if (inputs.m_POV == -1 & inputs.m_leftY == 0 & inputs.m_rightY == 0){
            // if we have no input, stop the motors.
            arm.armMotor.stopMotor();
            arm.intakeArmMotor.stopMotor();
            if(!hold && arm.goalLevel == 0){
                // if we have no input, not already holding, and are not using dp to go somewhere, start holding
                arm.holdAng[0] = arm.getArmEncoder();
                arm.holdAng[1] = arm.getIntakeEncoder();
                hold = true;
            }
            //no input, hold
        } else if(inputs.m_leftY != 0 | inputs.m_rightY != 0){
            // if we have a controller, have input on the sticks, we are manually moving. This should automatically override dp movement.
            arm.goalLevel = 0;
            // manual arming
            // moving, no hold.
            // if we are using sticks we need to stop the dp movement, manual should ovverride it.
            hold = false;
            arm.armMotor.set(inputs.m_leftY*0.2);
            // tolderance so we dont kill the intake:
            if (arm.getIntakeEncoder() <= 78 & inputs.m_rightY < 0) {
                arm.intakeArmMotor.stopMotor();
            } else {
                arm.intakeArmMotor.set(inputs.m_rightY*0.3);
            }
        }
        // grabbing cube
        if (inputs.m_LeftBumper) {
            arm.grabObject(true);
        } else if (inputs.m_RightBumper) {
            arm.placeObject(false);
        }

        // Grabbing cone
        if (inputs.m_LeftTriggerAxis > 0.1) {
            arm.placeObject(true);
        } else {
            if (inputs.m_RightTriggerAxis > 0.1) {
                arm.grabObject(false);
            } else {
                arm.softStop();
            }
        }

        // update arm
        arm.runArm(hold);

    }
}