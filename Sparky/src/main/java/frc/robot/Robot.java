
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivetrain.SwerveKinematics;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private static boolean holding = false;
	private static Inputs inputs = new Inputs(2);
    private static final int DRIVE_CONTROLLER_ID = 0;
    private static final int MANIP_CONTROLLER_ID = 1;
	private static double[] dp = {0, 0};
    CaptureReplay captureReplay = new CaptureReplay();

	private static SwerveKinematics chassis = new SwerveKinematics();
	private static Limelight limelight = new Limelight();
	private static Arm arm = new Arm();
	private static Intake intake = new Intake();

	private static boolean inAuto = false;

	private static boolean isFMSAttached = false;
	private static Alliance alliance = Alliance.Invalid;
	private static double matchTime = 0;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {

		getDSData();

		chassis.zeroGyro();
		chassis.configPIDS();
		captureReplay.setupDropdowns();

        SmartDashboard.putBoolean("autoMode", true);

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

		chassis.updateOdometry();

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

		chassis.configPIDS();
        captureReplay.closeFile();

		inputs.nullControls();

        captureReplay.autoFinished = false;

		inAuto = true;
		holding = false;
		arm.setLevel(0);

		chassis.zeroGyro();

		LED.currentColor = LED.RAINBOW_rainbowPallete;

		matchTime = 15;

		captureReplay.setupPlayback();

	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

		updateMatchTime();

        if (captureReplay.autoFinished) {
            inputs.nullControls();
        } else {
            inputs = captureReplay.readFile();
            RunControls();
        }
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {

		getDSData();

		inputs.nullControls();

		holding = false;
		inAuto = false;

        captureReplay.autoFinished = false;

		if (SmartDashboard.getBoolean("resetAngleOffsets", false)) {
			chassis.fixOffsets();
			System.out.println("Reset Offsets");
			SmartDashboard.putBoolean("resetAngleOffsets", false);
		}

		arm.setLevel(0);

		chassis.configPIDS();
		chassis.configEncoders();

        matchTime = 135;

	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		updateMatchTime();
		inputs.getControls();

        if (limelight.seesTag) {
            chassis.correctOdometry(limelight.position, Timer.getFPGATimestamp());
        }

		RunControls();
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		inputs.Rumbles[DRIVE_CONTROLLER_ID].setRumble(RumbleType.kBothRumble, 0);
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {

        getDSData();

		chassis.configPIDS();

		inputs.nullControls();

		inAuto = true;
		holding = false;
		arm.setLevel(0);

		chassis.zeroGyro();

		LED.currentColor = LED.RAINBOW_rainbowPallete;

		matchTime = 15;
        
        captureReplay.setupRecording();

    }

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {

        if (matchTime > 0) {

            inputs.getControls();
            
            captureReplay.record(inputs);

            RunControls();

            updateMatchTime();
        }

    }

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {}

	void RunControls() {

		if (alliance == Alliance.Blue) {
			LED.currentColor = LED.FIXEDPATTERN_waveOcean;
		} else if (alliance == Alliance.Red) {
			LED.currentColor = LED.FIXEDPATTERN_waveLava;
		} else {
			LED.currentColor = LED.FIXEDPATTERN_waveOcean;
		}

		if (matchTime < 31 & !inAuto) {
        	LED.currentColor = LED.MULTICOLOR_bpm;
        }
        
        if(inAuto) {
            LED.currentColor = LED.RAINBOW_rainbowPallete;
        }

		ExecuteDriveControls(((alliance == Alliance.Red) & inAuto) ? -1 : 1);
		ExecuteManipControls();

		LED.setColor();

	}

	void ExecuteDriveControls(int invert) {
		// Drive swerve chassis with joystick deadbands
        if (!DriverStation.isJoystickConnected(0)) {
            chassis.stop();
        } else {
            if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftX != 0 | inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY != 0) {
                if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].LeftTriggerAxis > 0 & inputs.ControllerInputs[DRIVE_CONTROLLER_ID].RightTriggerAxis > 0) {
                    chassis.drive(invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftX*0.2, inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY*0.2, invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightX*0.2*1.5);
                    if (!inAuto) {
                        inputs.Rumbles[DRIVE_CONTROLLER_ID].setRumble(RumbleType.kBothRumble, 0.2);
                    }
                    LED.currentColor = LED.SOLID_red;
                } else if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].LeftTriggerAxis > 0 | inputs.ControllerInputs[DRIVE_CONTROLLER_ID].RightTriggerAxis > 0) {
                    chassis.drive(invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftX*0.4, inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY*0.4, invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightX*0.4*1.5);
                    if (!inAuto) {
                        inputs.Rumbles[DRIVE_CONTROLLER_ID].setRumble(RumbleType.kBothRumble, 0.1);
                    }
                    LED.currentColor = LED.SOLID_redOrange;
                } else {
                    inputs.Rumbles[DRIVE_CONTROLLER_ID].setRumble(RumbleType.kBothRumble, 0);
                    chassis.drive(invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftX, inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY, invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightX*1.5);
                }
            } else {
                // D-Pad driving slowly
                inputs.Rumbles[DRIVE_CONTROLLER_ID].setRumble(RumbleType.kBothRumble, 0);
                if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].POV != -1) {
                    dp = Utils.getDirectionalPadValues(inputs.ControllerInputs[DRIVE_CONTROLLER_ID].POV);
                    chassis.drive(invert*dp[0] / 3, dp[1] / 3, invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightX);
                } else if (Math.abs(inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightX) > 0){
                    chassis.drive(0, 0, invert*inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightX);
                } else {
                    //Reset field orientation if we aren't moving the chassis
                    chassis.stop();
                }
            }

            chassis.relativeMode = inputs.ControllerInputs[DRIVE_CONTROLLER_ID].LeftBumper; 

        }

		if (limelight.seesTag & (matchTime == 0)) {
			chassis.navxGyro.setAngleAdjustment(chassis.robotRotation.minus(limelight.position.getRotation()).getDegrees());
		}

        // Line up with nearest cube grid
        // if (inputs.XButton) {
        //     double[] speeds = limelight.goToTarget(limelight.goToTag(), chassis.navxGyro.getYaw(), alliance);
        //     chassis.drive(speeds[0], speeds[1], speeds[2]);
        //     LED.currentColor = LED.SOLIwhite;
        // }

        if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].YButton) {
            chassis.configEncoders();
        }
	}

	void ExecuteManipControls() {
		// dP for controlling arm levels
        switch (inputs.ControllerInputs[MANIP_CONTROLLER_ID].POV) {
            case 0:
                // up - High
                arm.setLevel(3);
                holding = false;
                break;
            case 270:
                // left - Mid
                arm.setLevel(2);
                holding = false;
                break;
            case 180:
                // down - Hybrid
                arm.setLevel(1);
                holding = false;
                break;
            case 90:
                // right - Dock
                arm.setLevel(5);
                holding = false;
                break;
    
        }

        if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].AButtonPressed) {
            if (intake.pcm.getCompressor()) {
                intake.pcm.disableCompressor();
            } else {
                intake.pcm.enableCompressorDigital();
            }
        }

        if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].BackButton) {
            // back - HP Station
            arm.setLevel(4);
            holding = false;
        }
        
        if(!DriverStation.isJoystickConnected(1)){
            arm.armMotor.stopMotor();
            arm.intakeMotor.stopMotor();
        } else if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].POV == -1 & inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY == 0 & inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightY == 0){
            // if we have no input, stop the motors.
            arm.armMotor.stopMotor();
            arm.intakeMotor.stopMotor();
            inputs.Rumbles[MANIP_CONTROLLER_ID].setRumble(RumbleType.kLeftRumble, 0);
            inputs.Rumbles[MANIP_CONTROLLER_ID].setRumble(RumbleType.kRightRumble, 0);
            if(!holding & arm.goalLevel == 0){
                // if we have no input, not already holding, and are not using dp to go somewhere, start holding
                arm.holdAng[0] = arm.getArmEncoder();
                arm.holdAng[1] = arm.getIntakeEncoder();
                holding = true;
            }
            //no input, hold
        } else if(inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY != 0 | inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightY != 0){
            // if we have a controller, have input on the sticks, we are manually moving. This should automatically override dp movement.
            arm.setLevel(0);
            // manual arming
            // moving, no hold.
            // if we are using sticks we need to stop the dp movement, manual should ovverride it.
            holding = false;
            // tolerances so we don't kill the intake:
            if ((arm.getIntakeEncoder() <= 78 & inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightY < 0) | (arm.getIntakeEncoder() >= 330 & inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightY > 0)) {
                arm.intakeMotor.stopMotor();
                if (!inAuto) {
                    inputs.Rumbles[MANIP_CONTROLLER_ID].setRumble(RumbleType.kRightRumble, 0.05);
                }
            } else {
                arm.intakeMotor.set(inputs.ControllerInputs[DRIVE_CONTROLLER_ID].rightY*0.3);
                inputs.Rumbles[MANIP_CONTROLLER_ID].setRumble(RumbleType.kRightRumble, 0);
            }

            // tolerances so we don't kill the arm:
            if ((arm.getArmEncoder() >= 320 & arm.getArmEncoder() < 350 & inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY > 0) | (arm.getArmEncoder() <= 5 & inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY < 0)) {
                arm.armMotor.stopMotor();
                if (!inAuto) {
                    inputs.Rumbles[MANIP_CONTROLLER_ID].setRumble(RumbleType.kLeftRumble, 0.05);
                }
            } else {
                arm.armMotor.set(inputs.ControllerInputs[DRIVE_CONTROLLER_ID].leftY*0.2);
                inputs.Rumbles[MANIP_CONTROLLER_ID].setRumble(RumbleType.kLeftRumble, 0);
            }
        }
        
        if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].LeftBumper) {
            intake.grabObject(true);
        } else if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].RightBumper) {
            intake.grabObject(false);
        }else {
            intake.stopMotors();
        }

        // Grabbing cone
        if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].LeftTriggerAxis > 0.1) {
            intake.placeObject(true);
            LED.currentColor = LED.SOLID_gold;
        } else {
            if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].RightTriggerAxis > 0.1) {
                intake.placeObject(false);
                LED.currentColor = LED.SOLID_gold;
            }
        }

        //led signals
        if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].YButton) {
            LED.currentColor = LED.STROBE_purple;
        }
        if (inputs.ControllerInputs[DRIVE_CONTROLLER_ID].XButton) {
            LED.currentColor = LED.STROBE_gold;
        }

        // update arm
        arm.runArm(holding);
	}

	void updateDashboard() {

        if (!isFMSAttached & isDisabled()) {
			SmartDashboard.putNumber("armA", arm.getArmEncoder());
        	SmartDashboard.putNumber("intakeA", arm.getIntakeEncoder());
		}
        Rotation2d[] encVals = chassis.absEncoderValues();

        SmartDashboard.putNumberArray("moduleStates", new double[] {encVals[0].getDegrees(),encVals[1].getDegrees(),encVals[2].getDegrees(),encVals[3].getDegrees()});
        SmartDashboard.putNumber("gyro", chassis.robotRotation.getDegrees());
        
        Pose2d position = chassis.odometry.getEstimatedPosition();
        SmartDashboard.putNumberArray("odometry", new double[] {position.getX(), position.getY(), position.getRotation().getDegrees()});

    }

	void getDSData() {
		isFMSAttached = DriverStation.isFMSAttached();
		alliance = DriverStation.getAlliance();
	}

	void updateMatchTime() {matchTime -= 0.02;}

}