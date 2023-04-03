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

    private boolean hold = false;
    private Controller controller = new Controller();
    private ControllerInputs inputs;
    private double[] dp = {0,0,0};

    private SwerveKinematics chassis = new SwerveKinematics();
    private Auto auto = new Auto();
    private Limelight limelight = new Limelight();
    private Arm arm = new Arm();
    private LED led = new LED();

    private final SendableChooser <String> autoDropdown = new SendableChooser <> ();

    private boolean inAuto = false;
    private boolean isFMSAttached = false;
    private Alliance alliance = Alliance.Invalid;
    private double matchTime = 0;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        getDSData();

        chassis.navxGyro.zeroYaw();
        chassis.navxGyro.calibrate();

        auto.closeFile();

        chassis.configPIDS();

        autoDropdown.setDefaultOption("Default", auto.kAutoroutineDefault);
        autoDropdown.addOption("Basic - Left", auto.basicLeft);
        autoDropdown.addOption("Basic - Mid", auto.basicMid);
        autoDropdown.addOption("Basic - Right", auto.basicRight);
        autoDropdown.addOption("Charge - Left", auto.chargeLeft);
        autoDropdown.addOption("Charge - Right", auto.chargeRight);
        SmartDashboard.putData("Auto Sequence", autoDropdown);

        chassis.configEncoders();

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

        getDSData();

        auto.closeFile();
        chassis.configPIDS();

        if (alliance == Alliance.Blue) {
            selectedAuto = autoDropdown.getSelected();
        } else {
            if (autoDropdown.getSelected() == "bbl") {
                selectedAuto = "bbr";
            } else if (autoDropdown.getSelected() == "bbr") {
                selectedAuto = "bbl";
            } else {
                selectedAuto = autoDropdown.getSelected();
            }
        }
        auto.setupPlayback(selectedAuto);

        inputs = Controller.nullControls();

        auto.autoFinished = false;
        inAuto = true;
        hold = false;
        autoLevel = false;
        arm.goalLevel = 0;

        chassis.navxGyro.zeroYaw();

        led.currentColor = LED.MULTICOLOR_sinelonPurpleGold;

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        updateMatchTime();

        if (auto.autoFinished) {
            inputs = Controller.nullControls();
            if ((matchTime <= 6) & (selectedAuto == "bbl" | selectedAuto == "bbr")) {
                chassis.drive(0, -0.07, 0);
            }
        } else {
            if ((matchTime <= 6) & (selectedAuto == "bbl" | selectedAuto == "bbr")) {
                chassis.drive(0, -0.07, 0);
            } else if (selectedAuto == "def") {
                chassis.drive(0, -0.1, 0);
            } else {
                inputs = auto.readFile();
                if (selectedAuto == "bbl" | selectedAuto == "bbr") {
                    inputs.d_leftY = -inputs.d_leftY;
                    if (inputs.d_POV == 0) {
                        inputs.d_POV = 180;
                    } else if (inputs.d_POV == 90) {
                        inputs.d_POV = 270;
                    } else if (inputs.d_POV == 180) {
                        inputs.d_POV = 0;
                    } else if (inputs.d_POV == 270) {
                        inputs.d_POV = 90;
                    }
                }
                RunControls();
            }
        }

    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {

        getDSData();

        auto.closeFile();
        inputs = Controller.nullControls();

        autoLevel = false;
        hold = false;
        inAuto = false;

        if (SmartDashboard.getBoolean("resetAngleOffsets", false)) {
            chassis.fixOffsets();
            System.out.println("Reset Offsets");
            SmartDashboard.putBoolean("resetAngleOffsets", false);
        }

        arm.goalLevel = 0;

        chassis.configPIDS();

        chassis.configEncoders();

    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        updateMatchTime();
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

        getDSData();

        // record a new auto sequence
        selectedAuto = autoDropdown.getSelected();
        inputs = Controller.nullControls();
        auto.setupRecording(selectedAuto);
        
        arm.goalLevel = 0;

        autoLevel = false;
        hold = false;
        inAuto = true;

    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

        updateMatchTime();

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

        if (alliance == Alliance.Blue) {
            led.currentColor = LED.FIXEDPATTERN_waveOcean;
        } else if (alliance == Alliance.Red) {
            led.currentColor = LED.FIXEDPATTERN_waveLava;
        } else {
            led.currentColor = LED.FIXEDPATTERN_waveOcean;
        }

        if (matchTime < 31 & !inAuto) {
            led.currentColor = LED.MULTICOLOR_bpmPurpleGold;
        }
        
        if(inAuto) {
            led.currentColor = LED.RAINBOW_rainbowPallete;
        }

        ExecuteDriveControls(((alliance == Alliance.Red) & inAuto) ? -1 : 1);
        ExecuteManipControls();

        led.setColor();
        
    }

    void ExecuteDriveControls(int invert) {

        // Drive swerve chassis with joystick deadbands
        if (!DriverStation.isJoystickConnected(0)) {
            chassis.stop();
        } else {
            if (inputs.d_leftX != 0 | inputs.d_leftY != 0) {
                if (inputs.d_LeftTriggerAxis > 0 & inputs.d_RightTriggerAxis > 0) {
                    chassis.drive(invert*inputs.d_leftX*0.2, inputs.d_leftY*0.2, invert*inputs.d_rightX*0.2*1.5);
                    if (!inAuto) {
                        controller.d_rumble.setRumble(RumbleType.kBothRumble, 0.2);
                    }
                    led.currentColor = LED.SOLID_red;
                } else if (inputs.d_LeftTriggerAxis > 0 | inputs.d_RightTriggerAxis > 0) {
                    chassis.drive(invert*inputs.d_leftX*0.4, inputs.d_leftY*0.4, invert*inputs.d_rightX*0.4*1.5);
                    if (!inAuto) {
                        controller.d_rumble.setRumble(RumbleType.kBothRumble, 0.1);
                    }
                    led.currentColor = LED.SOLID_redOrange;
                } else {
                    controller.d_rumble.setRumble(RumbleType.kBothRumble, 0);
                    chassis.drive(invert*inputs.d_leftX, inputs.d_leftY, invert*inputs.d_rightX*1.5);
                }
            } else {
                // D-Pad driving slowly
                controller.d_rumble.setRumble(RumbleType.kBothRumble, 0);
                if (inputs.d_POV != -1) {
                    dp = Utils.getDirectionalPadValues(inputs.d_POV);
                    chassis.drive(invert*dp[0] / 3, dp[1] / 3, invert*inputs.d_rightX);
                } else if (Math.abs(inputs.d_rightX) > 0){
                    chassis.drive(0, 0, invert*inputs.d_rightX);
                } else {
                    //Reset field orientation if we aren't moving the chassis
                    if (inputs.d_AButton) {
                        chassis.navxGyro.zeroYaw();
                    }
                    chassis.stop();
                }
            }

            chassis.relativeMode = inputs.d_LeftBumper; 

        }


        // toggle auto level (only for autonomous)
        if (inputs.d_StartButton & inAuto) {
            autoLevel = true;
        }

        // if we're auto leveling, move to work
        if (autoLevel) {
            chassis.drive(0, Leveling.getBalanced(chassis.navxGyro.getPitch()+Leveling.PITCH_OFFSET), 0);
        }

        // Line up with nearest cube grid
        if (inputs.d_XButton) {
            // double[] speeds = limelight.alignWithTag(chassis.navxGyro.getYaw(), alliance);
            // chassis.drive(speeds[0], speeds[1], speeds[2]);
            double[] speeds = limelight.goToTarget(limelight.goToTag(), chassis.navxGyro.getYaw(), alliance);
            chassis.drive(speeds[0], speeds[1], speeds[2]);
            led.currentColor = LED.SOLID_white;
        }

        if (inputs.d_YButton) {
            chassis.configEncoders();
        }
    }


    //Manip Controls
    void ExecuteManipControls() {
        // dP for controlling arm levels
        switch (inputs.m_POV) {
            case 0:
                // up - High
                arm.setCurrentLevel(3);
                hold = false;
                break;
            case 270:
                // left - Mid
                arm.setCurrentLevel(2);
                hold = false;
                break;
            case 180:
                // down - Hybrid
                arm.setCurrentLevel(1);
                hold = false;
                break;
            case 90:
                // right - Dock
                arm.setCurrentLevel(5);
                hold = false;
                break;
    
        }

        if (inputs.m_AButtonPressed) {
            if (arm.pcm.getCompressor()) {
                arm.pcm.disableCompressor();
            } else {
                arm.pcm.enableCompressorDigital();
            }
        }

        if (inputs.m_BackButton) {
            // back - HP Station
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
            controller.m_rumble.setRumble(RumbleType.kLeftRumble, 0);
            controller.m_rumble.setRumble(RumbleType.kRightRumble, 0);
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
                if (!inAuto) {
                    controller.m_rumble.setRumble(RumbleType.kRightRumble, 0.05);
                }
            } else {
                arm.intakeArmMotor.set(inputs.m_rightY*0.3);
                controller.m_rumble.setRumble(RumbleType.kRightRumble, 0);
            }

            // tolerances so we don't kill the arm:
            if ((arm.getArmEncoder() >= 320 & arm.getArmEncoder() < 350 & inputs.m_leftY > 0) | (arm.getArmEncoder() <= 5 & inputs.m_leftY < 0)) {
                arm.armMotor.stopMotor();
                if (!inAuto) {
                    controller.m_rumble.setRumble(RumbleType.kLeftRumble, 0.05);
                }
            } else {
                arm.armMotor.set(inputs.m_leftY*0.2);
                controller.m_rumble.setRumble(RumbleType.kLeftRumble, 0);
            }
        }
        
        if (inputs.m_LeftBumper) {
            arm.grabObject(true);
        } else if (inputs.m_RightBumper) {
            arm.grabObject(false);
        }else {
            arm.leftWheels.stopMotor();
            arm.rightWheels.stopMotor();
        }

        // Grabbing cone
        if (inputs.m_LeftTriggerAxis > 0.1) {
            arm.placeObject(true);
            led.currentColor = LED.SOLID_gold;
        } else {
            if (inputs.m_RightTriggerAxis > 0.1) {
                arm.placeObject(false);
                led.currentColor = LED.SOLID_gold;
            }
        }

        //led signals
        if (inputs.m_YButton) {
            led.currentColor = LED.COLORONEPATTERN_strobePurple;
        }
        if (inputs.m_XButton) {
            led.currentColor = LED.FIXEDPATTERN_strobeGold;
        }

        // update arm
        arm.runArm(hold);

    }

    void updateDashboard() {

        if (!isFMSAttached & isDisabled()) {
           double[] encVals = chassis.encoderValues();
            SmartDashboard.putNumber("frontLeft", encVals[0]);
            SmartDashboard.putNumber("frontRight", encVals[1]);
            SmartDashboard.putNumber("backLeft", encVals[2]);
            SmartDashboard.putNumber("backRight", encVals[3]);
        }

        SmartDashboard.putNumber("armA", arm.getArmEncoder());
        SmartDashboard.putNumber("intakeA", arm.getIntakeEncoder());
        SmartDashboard.putNumber("pitch", chassis.navxGyro.getPitch()+Leveling.PITCH_OFFSET);

        SmartDashboard.putNumber("CAN Uilization", Math.floor(RobotController.getCANStatus().percentBusUtilization*100));

    }

    void getDSData() {
        isFMSAttached = DriverStation.isFMSAttached();
        alliance = DriverStation.getAlliance();
    }

    void updateMatchTime() {matchTime = DriverStation.getMatchTime();}
}