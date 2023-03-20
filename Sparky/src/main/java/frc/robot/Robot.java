// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Otters: 3229 Programming Sub-Team

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    private String selectedAuto = "";
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

    public final SendableChooser <String> autoDropdown = new SendableChooser <> ();

    double bufferZone = 0.1;
    boolean controllerError = false;
    boolean hasMovedArmManuallyYet = false;
    boolean inAuto = false;
    boolean onChargeStation = false;

    boolean manualIntakeToggle = true;
    boolean lastPressedA = false;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        chassis.navxGyro.zeroYaw();
        chassis.navxGyro.calibrate();

        auto.closeFile();

        chassis.configPIDS();

        autoDropdown.setDefaultOption("Right - High Cube - Taxi", auto.kTestAuto);
        autoDropdown.addOption("Mid - High Cube - Engage", auto.kMidAuto);
        autoDropdown.addOption("Left - High Cube - Taxi", auto.kLeftAuto);
        SmartDashboard.putData("Auto Sequence", autoDropdown);

        chassis.configEncoders();

        SmartDashboard.putNumber("kP", chassis.anglePID[0]);
        SmartDashboard.putNumber("kI", chassis.anglePID[1]);
        SmartDashboard.putNumber("kD", chassis.anglePID[2]);

        // Default LED pattern
        LED.setColor(LED.MULTICOLOR_twinklePurpleGold);

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

        updateDashboard();

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

        auto.closeFile();
        chassis.configPIDS();

        selectedAuto = autoDropdown.getSelected();
        auto.setupPlayback(selectedAuto);

        inputs = Controller.nullControls();

        autoLevel = false;

        auto.autoFinished = false;
        hold = false;
        inAuto = true;

        LED.setColor(LED.MULTICOLOR_sinelonPurpleGold);

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        if (auto.autoFinished) {
            inputs = Controller.nullControls();
        } else {
            inputs = auto.readFile();
            RunControls();
        }

    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {

        auto.closeFile();

        inputs = Controller.nullControls();

        autoLevel = false;
        hold = false;
        inAuto = false;

        if (dash.readBool("resetAngleOffsets")) {
            chassis.fixOffsets();
            System.out.println("Reset Offsets");
            dash.putBool("resetAngleOffsets", false);
        }

        arm.goalLevel = 0;

        hasMovedArmManuallyYet = false;

        chassis.anglePID[0] = SmartDashboard.getNumber("kP", 0);
        chassis.anglePID[1] = SmartDashboard.getNumber("kI", 0);
        chassis.anglePID[2] = SmartDashboard.getNumber("kD", 0);

        chassis.configPIDS();

        chassis.configEncoders();

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
        selectedAuto = autoDropdown.getSelected();
        inputs = Controller.nullControls();
        auto.setupRecording(selectedAuto);

        autoLevel = false;
        hold = false;
        inAuto = true;

    }

    //Nathan D was here
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

        if (DriverStation.getMatchTime() < 31 & !inAuto) {
            LED.setColor(LED.MULTICOLOR_twinklePurpleGold);
        }
        if (DriverStation.getAlliance() == Alliance.Blue) {
            LED.setColor(LED.FIXEDPATTERN_waveOcean);
        }
        if (DriverStation.getAlliance() == Alliance.Red) {
            LED.setColor(LED.FIXEDPATTERN_waveLava);
        }

        ExecuteDriveControls();
        ExecuteManipControls();
        
    }

    void ExecuteDriveControls() {

        // Drive swerve chassis with joystick deadbands
        if (!DriverStation.isJoystickConnected(0)) {
            chassis.stop();
        } else {
            if (Math.abs(inputs.d_leftX) > 0 | Math.abs(inputs.d_leftY) > 0) {
                if (inputs.d_LeftTriggerAxis > 0 & inputs.d_RightTriggerAxis > 0) {
                    chassis.drive(inputs.d_leftX*0.2, inputs.d_leftY*0.2, inputs.d_rightX*0.2);
                    controller.d_rumble.setRumble(RumbleType.kBothRumble, 0.4);
                    LED.setColor(LED.SOLID_red);
                } else if (inputs.d_LeftTriggerAxis > 0 | inputs.d_RightTriggerAxis > 0) {
                    chassis.drive(inputs.d_leftX*0.4, inputs.d_leftY*0.4, inputs.d_rightX*0.4);
                    controller.d_rumble.setRumble(RumbleType.kBothRumble, 0.2);
                    LED.setColor(LED.SOLID_redOrange);
                } else {
                    chassis.drive(inputs.d_leftX, inputs.d_leftY, inputs.d_rightX);
                }
            } else {
                // D-Pad driving slowly

                if (inputs.d_POV != -1) {
                    dp = utils.getDirectionalPadValues(inputs.d_POV);
                    chassis.drive(dp[0] / 3, dp[1] / 3, inputs.d_rightX);
                } else if (Math.abs(inputs.d_rightX) > 0){
                    chassis.drive(0, 0, inputs.d_rightX);
                } else {
                    chassis.stop();
                }
            }

            chassis.relativeMode = inputs.d_LeftBumper;

        }


        // toggle auto level (only for autonomous)
        if (inputs.d_StartButton & inAuto) {
            autoLevel = true;
        }

        if ((chassis.navxGyro.getPitch() <= -3) & onChargeStation == false & inAuto) {
            onChargeStation = true;
        }

        // if we're auto leveling, move to work
        if (autoLevel) {
            chassis.drive(0, Leveling.getBalanced(chassis.navxGyro.getPitch(), onChargeStation), 0);
        }

        // Reset field orientation
        if (inputs.d_AButton) {
            chassis.navxGyro.zeroYaw();
        }

        // Line up with nearest cube grid
        if (inputs.d_XButton) {
            double[] speeds = limelight.alignWithTag(chassis.navxGyro.getYaw(), DriverStation.getAlliance());
            chassis.drive(speeds[0], speeds[1], speeds[2]);
        }

    }


    //Manip Controls
    void ExecuteManipControls() {
        // dP for controlling arm levels
        switch (inputs.m_POV) {
            case 0:
                // up - High
                hasMovedArmManuallyYet = false;
                arm.setCurrentLevel(3);
                hold = false;
                break;
            case 270:
                // left - Mid
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
            case 90:
                // right - Dock
                hasMovedArmManuallyYet = false;
                arm.setCurrentLevel(4);
                hold = false;
                break;
    
        }

        if (inputs.m_BackButton) {
            // back - HP Station
            hasMovedArmManuallyYet = false;
            arm.setCurrentLevel(4);
            hold = false;
        }
        
        if(!DriverStation.isJoystickConnected(1)){
            arm.armMotor.stopMotor();
            arm.intakeArmMotor.stopMotor();
        } else if (inputs.m_POV == -1 & inputs.m_leftY == 0 & inputs.m_rightY == 0){
            // if we have no input, stop the motors.
            arm.armMotor.stopMotor();
            arm.intakeArmMotor.stopMotor();
            if(!hold & arm.goalLevel == 0){
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
            // tolerances so we don't kill the intake:
            if ((arm.getIntakeEncoder() <= 78 & inputs.m_rightY < 0) | (arm.getIntakeEncoder() >= 330 & inputs.m_rightY > 0)) {
                arm.intakeArmMotor.stopMotor();
            } else {
                arm.intakeArmMotor.set(inputs.m_rightY*0.3);
            }

            // tolerances so we don't kill the arm:
            if ((arm.getArmEncoder() >= 320 & arm.getArmEncoder() < 350 & inputs.m_leftY > 0) | (arm.getArmEncoder() <= 5 & inputs.m_leftY < 0)) {
                arm.armMotor.stopMotor();
            } else {
                arm.armMotor.set(inputs.m_leftY*0.2);
            }
        }
        
        // grab something
        if (inputs.m_LeftBumper & !manualIntakeToggle) {
            // grab based on color sensor
            arm.grabObject();
        } else if (inputs.m_RightBumper & !manualIntakeToggle) {
            // Place based on what we are holding, if no cube we do cone, etc.
            arm.placeObject();
        } else if (inputs.m_LeftBumper) {
            arm.grabObject(true);
        } else if (inputs.m_RightBumper) {
            arm.grabObject(false);
        } else {
            arm.leftWheels.stopMotor();
            arm.rightWheels.stopMotor();
        }

        // turn on manual mode
        // if (inputs.m_AButton & !lastPressedA) {
        //     manualIntakeToggle = !manualIntakeToggle;
        //     lastPressedA = true;
        // } else {
        //     lastPressedA = false;
        // }

        // Grabbing cone
        if (inputs.m_LeftTriggerAxis > 0.1 & manualIntakeToggle) {
            arm.placeObject(true);
            LED.setColor(LED.SOLID_gold);
        } else {
            if (inputs.m_RightTriggerAxis > 0.1 & manualIntakeToggle) {
                arm.placeObject(false);
                LED.setColor(LED.SOLID_gold);
            }
        }

        // Request Cube
        if (inputs.m_AButton) {
            LED.setColor(LED.COLORONEPATTERN_strobePurple);
        }
        
        // Request Cone
        if (inputs.m_BButton) {
            LED.setColor(LED.FIXEDPATTERN_strobeGold);
        }

        // update arm
        arm.runArm(hold);

    }

    void updateDashboard() {

        if (!DriverStation.isFMSAttached() | DriverStation.isDisabled()) {
           double[] encVals = chassis.encoderValues();
            dash.putNumber("frontLeft", encVals[0]);
            dash.putNumber("frontRight", encVals[1]);
            dash.putNumber("backLeft", encVals[2]);
            dash.putNumber("backRight", encVals[3]);
        }

        dash.putNumber("CAN Uilization", Math.floor(RobotController.getCANStatus().percentBusUtilization*100));

    }
}