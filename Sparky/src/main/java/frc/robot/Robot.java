// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivetrain.SwerveKinematics;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
	public class Robot extends TimedRobot {

	private static String selectedAuto = "";
	private static boolean autoLeveling = false;

	private static boolean holding = false;
	private static Controller controller = new Controller();
	private static ControllerInputs inputs;
	private static double[] dp = {0, 0};

	private static SwerveKinematics chassis = new SwerveKinematics();
	private static Limelight limelight = new Limelight();
	private static Arm arm = new Arm();
	private static Intake intake = new Intake();

	private final SendableChooser <String> autoDropdown = new SendableChooser <> ();

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

		autoDropdown.setDefaultOption("Default", "def");
        autoDropdown.addOption("Basic - Left", "bl");
        autoDropdown.addOption("Basic - Mid", "bm");
        autoDropdown.addOption("Basic - Right", "br");
        autoDropdown.addOption("Charge - Left", "cl");
        autoDropdown.addOption("Charge - Right", "cr");
        SmartDashboard.putData("Auto Sequence", autoDropdown);

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

		inputs = Controller.nullControls();

		inAuto = true;
		holding = false;
		autoLeveling = false;
		arm.setLevel(0);

		chassis.zeroGyro();

		LED.currentColor = LED.RAINBOW_rainbowPallete;

		matchTime = 15;
		
		Auto.selectAuto(selectedAuto, chassis.odometry::getEstimatedPosition, chassis::drive);

	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {

		updateMatchTime();

		Auto.autoCommand.execute();
		// inputs = auto.read();

	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {

		getDSData();

		inputs = Controller.nullControls();

		autoLeveling = false;
		holding = false;
		inAuto = false;

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
		inputs = controller.getControls();

        if (limelight.seesTag) {
            chassis.correctOdometry(limelight.position, Timer.getFPGATimestamp());
        }

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
	public void testInit() {}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}

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
            if (inputs.d_leftX != 0 | inputs.d_leftY != 0) {
                if (inputs.d_LeftTriggerAxis > 0 & inputs.d_RightTriggerAxis > 0) {
                    chassis.drive(invert*inputs.d_leftX*0.2, inputs.d_leftY*0.2, invert*inputs.d_rightX*0.2*1.5);
                    if (!inAuto) {
                        controller.d_rumble.setRumble(RumbleType.kBothRumble, 0.2);
                    }
                    LED.currentColor = LED.SOLID_red;
                } else if (inputs.d_LeftTriggerAxis > 0 | inputs.d_RightTriggerAxis > 0) {
                    chassis.drive(invert*inputs.d_leftX*0.4, inputs.d_leftY*0.4, invert*inputs.d_rightX*0.4*1.5);
                    if (!inAuto) {
                        controller.d_rumble.setRumble(RumbleType.kBothRumble, 0.1);
                    }
                    LED.currentColor = LED.SOLID_redOrange;
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
                    chassis.stop();
                }
            }

            chassis.relativeMode = inputs.d_LeftBumper; 

        }

		if (limelight.seesTag & (matchTime%2 == 0)) {
			chassis.navxGyro.setAngleAdjustment(chassis.robotRotation.minus(limelight.position.getRotation()).getDegrees());
		}

        // toggle auto level (only for autonomous)
        if (inputs.d_StartButton & inAuto) {
            autoLeveling = true;
        }

        // if we're auto leveling, move to work
        if (autoLeveling) {
            chassis.drive(0, Leveling.getBalanced(chassis.getPitch()+Leveling.PITCH_OFFSET), 0);
        }

        // Line up with nearest cube grid
        // if (inputs.d_XButton) {
        //     double[] speeds = limelight.goToTarget(limelight.goToTag(), chassis.navxGyro.getYaw(), alliance);
        //     chassis.drive(speeds[0], speeds[1], speeds[2]);
        //     LED.currentColor = LED.SOLID_white;
        // }

        if (inputs.d_YButton) {
            chassis.configEncoders();
        }
	}

	void ExecuteManipControls() {
		// dP for controlling arm levels
        switch (inputs.m_POV) {
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

        if (inputs.m_AButtonPressed) {
            if (intake.pcm.getCompressor()) {
                intake.pcm.disableCompressor();
            } else {
                intake.pcm.enableCompressorDigital();
            }
        }

        if (inputs.m_BackButton) {
            // back - HP Station
            arm.setLevel(4);
            holding = false;
        }
        
        if(!DriverStation.isJoystickConnected(1)){
            arm.armMotor.stopMotor();
            arm.intakeMotor.stopMotor();
        } else if (inputs.m_POV == -1 & inputs.m_leftY == 0 & inputs.m_rightY == 0){
            // if we have no input, stop the motors.
            arm.armMotor.stopMotor();
            arm.intakeMotor.stopMotor();
            controller.m_rumble.setRumble(RumbleType.kLeftRumble, 0);
            controller.m_rumble.setRumble(RumbleType.kRightRumble, 0);
            if(!holding & arm.goalLevel == 0){
                // if we have no input, not already holding, and are not using dp to go somewhere, start holding
                arm.holdAng[0] = arm.getArmEncoder();
                arm.holdAng[1] = arm.getIntakeEncoder();
                holding = true;
            }
            //no input, hold
        } else if(inputs.m_leftY != 0 | inputs.m_rightY != 0){
            // if we have a controller, have input on the sticks, we are manually moving. This should automatically override dp movement.
            arm.setLevel(0);
            // manual arming
            // moving, no hold.
            // if we are using sticks we need to stop the dp movement, manual should ovverride it.
            holding = false;
            // tolerances so we don't kill the intake:
            if ((arm.getIntakeEncoder() <= 78 & inputs.m_rightY < 0) | (arm.getIntakeEncoder() >= 330 & inputs.m_rightY > 0)) {
                arm.intakeMotor.stopMotor();
                if (!inAuto) {
                    controller.m_rumble.setRumble(RumbleType.kRightRumble, 0.05);
                }
            } else {
                arm.intakeMotor.set(inputs.m_rightY*0.3);
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
            intake.grabObject(true);
        } else if (inputs.m_RightBumper) {
            intake.grabObject(false);
        }else {
            intake.stopMotors();
        }

        // Grabbing cone
        if (inputs.m_LeftTriggerAxis > 0.1) {
            intake.placeObject(true);
            LED.currentColor = LED.SOLID_gold;
        } else {
            if (inputs.m_RightTriggerAxis > 0.1) {
                intake.placeObject(false);
                LED.currentColor = LED.SOLID_gold;
            }
        }

        //led signals
        if (inputs.m_YButton) {
            LED.currentColor = LED.STROBE_purple;
        }
        if (inputs.m_XButton) {
            LED.currentColor = LED.STROBE_gold;
        }

        // update arm
        arm.runArm(holding);
	}

	void updateDashboard() {

        if (!isFMSAttached & isDisabled()) {
        	Rotation2d[] encVals = chassis.absEncoderValues();
            SmartDashboard.putNumber("frontLeft", encVals[0].getDegrees());
            SmartDashboard.putNumber("frontRight", encVals[1].getDegrees());
            SmartDashboard.putNumber("backLeft", encVals[2].getDegrees());
            SmartDashboard.putNumber("backRight", encVals[3].getDegrees());

			SmartDashboard.putNumber("armA", arm.getArmEncoder());
        	SmartDashboard.putNumber("intakeA", arm.getIntakeEncoder());
		}

        SmartDashboard.putNumber("CAN Uilization", Math.floor(RobotController.getCANStatus().percentBusUtilization*100));
    }

	void getDSData() {
		isFMSAttached = DriverStation.isFMSAttached();
		alliance = DriverStation.getAlliance();
	}

	void updateMatchTime() {matchTime -= 0.02;}

}