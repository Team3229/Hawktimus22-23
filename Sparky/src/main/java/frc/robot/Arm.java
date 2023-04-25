package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;

public class Arm {
 
    CANSparkMax armMotor;
    CANSparkMax armMotorFollower;
    CANSparkMax intakeMotor;

    CANCoder armEncoder;
    CANCoder intakeEncoder;

    public double[] holdAng = {0,0};

    public int goalLevel = 0;

    private boolean holdingCube = true;
    private boolean holdingCone = false;

    // CAN IDs
    private static final int ARM_ID = 16;
    private static final int ARM_FOLLOW_ID = 17;
    private static final int INTAKE_ID = 18;
    private static final int ARM_ENCODER_ID = 19;
    private static final int INTAKE_ENCODER_ID = 20;

    // Encoder offsets
    private static final double ARM_OFFSET = 96.7890625;
    private static final double INTAKE_OFFSET = 142.470703125;

    // Constants
    private static final double ARM_SPEED = 0.4;
    private static final double ARM_SPEED_2 = 0.2;
    private static final double ARM_SPEED_3 = 0.08;

    private static final double INTAKE_SPEED = 0.3;
    private static final double INTAKE_SPEED_2 = 0.25;
    private static final double INTAKE_SPEED_3 = 0.05;

    private static final int ARM_2_POS = 7;
    private static final int ARM_3_POS = 1;

    private static final double HIGH_CONE = 204.533203125;
    private static final double HIGH_CUBE = 224.033203125;
    private static final double MID_CONE = 204.697265625;
    private static final double MID_CUBE = 221.044921875;
    private static final double HYBRID = 325.8984375;
    private static final double PLAYER = 247.587890625;
    private static final double DOCK = 17;

    private static final double IHIGH_CONE = 238.8640625;
    private static final double IHIGH_CUBE = 204.515625;
    private static final double IMID_CONE = 289.599609375;
    private static final double IMID_CUBE = 289.248046875;
    private static final double IHYBRID = 115.576171875;
    private static final double IPLAYER = 183.1640625;
    private static final double IDOCK = 330;

    Arm() {
        // Arm
        armMotor = new CANSparkMax(ARM_ID, MotorType.kBrushless);
        armMotorFollower = new CANSparkMax(ARM_FOLLOW_ID, MotorType.kBrushless);
        armMotor.setInverted(false);
        armMotorFollower.follow(armMotor, true);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotorFollower.setIdleMode(IdleMode.kBrake);

        intakeMotor = new CANSparkMax(INTAKE_ID, MotorType.kBrushless);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        // Encoders
        armEncoder = new CANCoder(ARM_ENCODER_ID);
        armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        armEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armEncoder.configSensorDirection(true);
        armEncoder.configMagnetOffset(ARM_OFFSET);
        armEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);

        intakeEncoder = new CANCoder(INTAKE_ENCODER_ID);
        intakeEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        intakeEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        intakeEncoder.configSensorDirection(true);
        intakeEncoder.configMagnetOffset(INTAKE_OFFSET);
        intakeEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);

        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        armMotorFollower.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    }

    public void setLevel(int level){goalLevel = level;}

    public double getArmEncoder() {

        return MathUtil.inputModulus(armEncoder.getAbsolutePosition(), 0, 360);

    }

    public double getIntakeEncoder() {

        return MathUtil.inputModulus(intakeEncoder.getAbsolutePosition(), 0, 360);

    }

    private double[] calculateArmOutputs(double aAngle, double iAngle, double th, double ih) {
        // returns the motor speed
        // taking the current angles, and the goal angles for intake and arm, calc what dir we need to move.
        double[] returningVal = {0, 0};
        // if the arm is lower than the lower bounds of the target, move it up, else down, if in tolerance do nothing.
        if (iAngle < ih-ARM_3_POS) {
            returningVal[1] = INTAKE_SPEED_3;
        } else if (iAngle > ih+ARM_3_POS) {
            returningVal[1] = -INTAKE_SPEED_3;
        }

        if (iAngle < ih-ARM_2_POS) {
            returningVal[1] = INTAKE_SPEED_2;
        } else if (iAngle > ih+ARM_2_POS) {
            returningVal[1] = -INTAKE_SPEED_2;
        }

        if (iAngle < ih-(ARM_2_POS+40)) {
            returningVal[1] = INTAKE_SPEED;
        } else if (iAngle > ih+(ARM_2_POS+40)) {
            returningVal[1] = -INTAKE_SPEED;
        }

        // fast deploy
        if (aAngle < 150 & !(goalLevel == 4)) {
            if (aAngle < th - 1) {
                returningVal[0] = 0.8;
            } else if (aAngle > th+1) {
                returningVal[0] = -0.8;
            }
            returningVal[1] = 0;
        } else {
            if (aAngle < th - ARM_3_POS) {
                returningVal[0] = ARM_SPEED_3;
            } else if (aAngle > th+ARM_3_POS) {
                returningVal[0] = -ARM_SPEED_3;
            }
            if (aAngle < th - ARM_2_POS) {
                returningVal[0] = ARM_SPEED_2;
            } else if (aAngle > th+ARM_2_POS) {
                returningVal[0] = -ARM_SPEED_2;
            }
            if (aAngle < th - (ARM_2_POS+20)) {
                returningVal[0] = ARM_SPEED;
            } else if (aAngle > th+(ARM_2_POS+20)) {
                returningVal[0] = -ARM_SPEED;
            }
            
        }
        // no break intake from hybrid/ pickup position
        if (aAngle > 290 & !(goalLevel == 1)) {
            returningVal[1] = 0;
        }
        return returningVal;
    }

    private double[] calculateArmLevel(int level){
        // get angles
        double armAngle = getArmEncoder();
        double intakeAngle = getIntakeEncoder();
        // switch over the level, 0 meaning we have no level and should not be moving and 1,2,3,4,5 for heights.
        // all handle cone/cube movement, returning the right values for each, returns the result of calcArmOutputs.
        // returns a double[] of values, arm then intake, 0 if were at the zone and a value otherwise to go in the right direction.
        switch(level){
            case 1:
                // hybrid
                return calculateArmOutputs(armAngle, intakeAngle, HYBRID, IHYBRID);
            case 2:
                // mid
                if (holdingCube) {
                    return calculateArmOutputs(armAngle, intakeAngle, MID_CUBE, IMID_CUBE);
                } else if (holdingCone) {
                    return calculateArmOutputs(armAngle, intakeAngle, MID_CONE, IMID_CONE);
                } else {
                    return calculateArmOutputs(armAngle, intakeAngle, MID_CUBE, IMID_CUBE);
                }
            case 3:
                // high
                if (holdingCube) {
                    return calculateArmOutputs(armAngle, intakeAngle, HIGH_CUBE, IHIGH_CUBE);
                } else if (holdingCone) {
                    return calculateArmOutputs(armAngle, intakeAngle, HIGH_CONE, IHIGH_CONE);
                } else {
                    return calculateArmOutputs(armAngle, intakeAngle, HIGH_CUBE, IHIGH_CUBE);
                }
            case 4:
                //dock
                return calculateArmOutputs(armAngle, intakeAngle, DOCK, IDOCK);
            case 5:
                //human player
                return calculateArmOutputs(armAngle, intakeAngle, PLAYER, IPLAYER);
            default:
                return new double[] {};
        }
    }

    void runArm(boolean hold) {
        if(!hold){
            // not holding, meaining we have dp movement or stick movement, so handle movement
            // Arm
            // if we have a goal level, meaning we have pressed the dp and want to be going somewhere,
            if (goalLevel != 0) {
                // calculate what dir we have to move the arm
                double[] armResults = calculateArmLevel(goalLevel);

                // if we have a direction to move the arm, move it. else stop it.
                if (armResults[0] != 0) {
                    armMotor.set(armResults[0]);
                } else {
                    armMotor.stopMotor();
                }

                // If we have somewhere to move the intake, move it else stop it
                if (armResults[1] != 0) {
                    intakeMotor.set(armResults[1]);
                } else {
                    intakeMotor.stopMotor();
                }

                // if both are zero, reset goal level because we're at the right location.
                if(armResults[1] == 0 & armResults[0] == 0){
                    goalLevel = 0;
                    hold = true;
                    holdPos();
                }
            }
        } else {
            holdPos();
        }
        
    }

    void holdPos() {
        if(getArmEncoder() > holdAng[0] & getArmEncoder() > 160){
            armMotor.set(-0.03);
        }
        if(getIntakeEncoder() > holdAng[1] & getArmEncoder() > 160){
            intakeMotor.set(-0.04);
        }
    }

}