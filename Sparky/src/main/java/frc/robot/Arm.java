//Otters: 3229 Programming Sub-Team

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Arm {
    /*
    1 low
    2 mid
    3 high
    4 human station
    */

    // Objects
    CANSparkMax armMotor;
    CANSparkMax armMotor2;
    CANSparkMax intakeArmMotor;
    CANSparkMax leftWheels;
    CANSparkMax rightWheels;
    Solenoid onSolenoid;
    Solenoid offSolenoid;
    PneumaticsControlModule pcm;
    Compressor compressor;
    CANCoder armEncoder;
    CANCoder intakeArmEncoder;
    RelativeEncoder leftWheelsEncoder;
    RelativeEncoder rightWheelsEncoder;

    // CAN IDs
    final int ARM_ID = 16;
    final int ARM_2_ID = 17;
    final int INTAKE_ARM_ID = 18;
    final int LEFT_HAND_ID = 14;
    final int RIGHT_HAND_ID = 15;
    final int ARM_ENCODER_ID = 19;
    final int INTAKE_ENCODER_ID = 20;

    // Encoder Offsets
    final double ARM_ENCODER_OFFSET = 96.7890625;
    final double INTAKE_ENCODER_OFFSET = 142.470703125;

    // Constants
    final double ARM_LENGTH = 0.8218;
    final double ARM_PIVOT_HEIGHT = 0.9836;
    final double IN_HAND_ROTATIONAL_SPEED = -0.2;
    final double OUT_HAND_ROTATIONAL_SPEED = 0.1;

    final double ARM_SPEED = 0.4;
    final double INTAKE_SPEED = 0.3;
    final double ARM_CLOSE_SPEED = 0.2;
    final double INTAKE_CLOSE_SPEED = 0.25;
    final double INTAKE_CLOSER_SPEED = 0.05;
    final double ARM_CLOSER_SPEED = 0.08;

    final double INTAKE_CLOSE_POS = 7;
    final double INTAKE_CLOSER_POS = 1;

    final double ARM_CLOSE_POS = 7;
    final double ARM_CLOSER_POS = 1;

    final double HIGH_CONE = 204.533203125; //DONE
    final double HIGH_CUBE = 224.033203125; //DONE
    final double MID_CONE = 204.697265625; //DONE
    final double MID_CUBE = 221.044921875; //DONE
    final double HYBRID = 325.8984375; //DONE
    final double DOCK = 10; //DONE
    final double IHIGH_CONE = 238.8640625; //DONE
    final double IHIGH_CUBE = 204.515625; //DONE
    final double IMID_CONE = 289.599609375; //DONE
    final double IMID_CUBE = 289.248046875; //DONE
    final double IHYBRID = 115.576171875; //DONE
    final double IDOCK = 330; //DONE

    final double PLAYER = 247.587890625;
    final double IPLAYER = 183.1640625;

    public double[] holdAng = {0,0};
    // Other Variables
    int goalLevel = 0;
    int armEncoderBuffer = 0;
    double armEncoderValue = 0;
    int intakeEncoderBuffer = 0;
    double intakeEncoderValue = 0;

    boolean holdingCube = true;
    boolean holdingCone = false;

    Arm() {
        
        // Arm
        armMotor = new CANSparkMax(ARM_ID, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(ARM_2_ID, MotorType.kBrushless);
        armMotor.setInverted(false);
        armMotor2.follow(armMotor, true);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);

        // Intake
        intakeArmMotor = new CANSparkMax(INTAKE_ARM_ID, MotorType.kBrushless);
        intakeArmMotor.setIdleMode(IdleMode.kBrake);
        leftWheels = new CANSparkMax(LEFT_HAND_ID, MotorType.kBrushless);
        rightWheels = new CANSparkMax(RIGHT_HAND_ID, MotorType.kBrushless);
        leftWheels.setInverted(false);
        rightWheels.setInverted(true);

        // Pnuematics
        pcm = new PneumaticsControlModule();
        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        onSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
        offSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

        pcm.clearAllStickyFaults();

        // Encoders
        armEncoder = new CANCoder(ARM_ENCODER_ID);
        armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        armEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armEncoder.configSensorDirection(true);
        armEncoder.configMagnetOffset(ARM_ENCODER_OFFSET);
        armEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);

        intakeArmEncoder = new CANCoder(INTAKE_ENCODER_ID);
        intakeArmEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        intakeArmEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        intakeArmEncoder.configSensorDirection(true);
        intakeArmEncoder.configMagnetOffset(INTAKE_ENCODER_OFFSET);
        intakeArmEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);

        leftWheelsEncoder = leftWheels.getEncoder();
        rightWheelsEncoder = rightWheels.getEncoder();

        leftWheelsEncoder.setPositionConversionFactor(360);
        rightWheelsEncoder.setPositionConversionFactor(360);

        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        armMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        intakeArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        intakeArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        leftWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        leftWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        intakeArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        rightWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        rightWheels.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    }
    
    double[] calculateArmLevel(int level){
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

    //fix 180 I put it there to make sure it was working and it isn't
    double[] calculateArmOutputs(double aAngle, double iAngle, double th, double ih) {
        // returns the motor speed
        // taking the current angles, and the goal angles for intake and arm, calc what dir we need to move.
        double[] returningVal = {0, 0};
        // if the arm is lower than the lower bounds of the target, move it up, else down, if in tolerance do nothing.
        if (iAngle < ih-INTAKE_CLOSER_POS) {
            returningVal[1] = INTAKE_CLOSER_SPEED;
        } else if (iAngle > ih+INTAKE_CLOSER_POS) {
            returningVal[1] = -INTAKE_CLOSER_SPEED;
        }

        if (iAngle < ih-INTAKE_CLOSE_POS) {
            returningVal[1] = INTAKE_CLOSE_SPEED;
        } else if (iAngle > ih+INTAKE_CLOSE_POS) {
            returningVal[1] = -INTAKE_CLOSE_SPEED;
        }

        if (iAngle < ih-(INTAKE_CLOSE_POS+40)) {
            returningVal[1] = INTAKE_SPEED;
        } else if (iAngle > ih+(INTAKE_CLOSE_POS+40)) {
            returningVal[1] = -INTAKE_SPEED;
        }

        // same here
        // fast deploy
        if (aAngle < 150 & !(goalLevel == 4)) {
            if (aAngle < th - 1) {
                returningVal[0] = 0.8;
            } else if (aAngle > th+1) {
                returningVal[0] = -0.8;
            }
            returningVal[1] = 0;
        } else {
            if (aAngle < th - ARM_CLOSER_POS) {
                returningVal[0] = ARM_CLOSER_SPEED;
            } else if (aAngle > th+ARM_CLOSER_POS) {
                returningVal[0] = -ARM_CLOSER_SPEED;
            }
            if (aAngle < th - ARM_CLOSE_POS) {
                returningVal[0] = ARM_CLOSE_SPEED;
            } else if (aAngle > th+ARM_CLOSE_POS) {
                returningVal[0] = -ARM_CLOSE_SPEED;
            }
            if (aAngle < th - (ARM_CLOSE_POS+20)) {
                returningVal[0] = ARM_SPEED;
            } else if (aAngle > th+(ARM_CLOSE_POS+20)) {
                returningVal[0] = -ARM_SPEED;
            }
            
        }
        // no break intake from hybrid/ pickup position
        if (aAngle > 290 & !(goalLevel == 1)) {
            returningVal[1] = 0;
        }

        return returningVal;
    }

    void grabObject(boolean cube){
        
        if (!cube) {
            offSolenoid.set(false);
            onSolenoid.set(true);
            leftWheels.set(IN_HAND_ROTATIONAL_SPEED*0.6);
            rightWheels.set(IN_HAND_ROTATIONAL_SPEED*0.6);
            holdingCone = true;
            holdingCube = false;
        } else {
            offSolenoid.set(true);
            onSolenoid.set(false);
            leftWheels.set(IN_HAND_ROTATIONAL_SPEED);
            rightWheels.set(IN_HAND_ROTATIONAL_SPEED);
            holdingCone = false;
            holdingCube = true;
        }
    }

    void placeObject(boolean cube){

        if(!cube){
            // move the pneumatic cone bits
            offSolenoid.set(false);
            onSolenoid.set(true);
            leftWheels.set(OUT_HAND_ROTATIONAL_SPEED*2.5);
            rightWheels.set(OUT_HAND_ROTATIONAL_SPEED*2.5);
        } else {
            // holding a cube
            leftWheels.set(OUT_HAND_ROTATIONAL_SPEED);
            rightWheels.set(OUT_HAND_ROTATIONAL_SPEED);
            offSolenoid.set(true);
            onSolenoid.set(false);  
        }
        
    }

    public double getArmEncoder() {

        if (armEncoderBuffer++ > 7) {
            armEncoderBuffer = 0;
            armEncoderValue = armEncoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(armEncoderValue, 0, 360);

    }

    public double getIntakeEncoder() {

        if (intakeEncoderBuffer++ > 7) {
            intakeEncoderBuffer = 0;
            intakeEncoderValue = intakeArmEncoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(intakeEncoderValue, 0, 360);

    }

    void setCurrentLevel(int level){goalLevel = level;}

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
                    intakeArmMotor.set(armResults[1]);
                } else {
                    intakeArmMotor.stopMotor();
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
        if(getArmEncoder() > holdAng[0] & getArmEncoder() > 10){
            armMotor.set(-0.03);
        }
        if(getIntakeEncoder() > holdAng[1]){
            intakeArmMotor.set(-0.04);
        }
    }

}