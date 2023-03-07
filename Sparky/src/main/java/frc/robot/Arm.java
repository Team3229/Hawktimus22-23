//Otters: 3229 Programming Sub-Team

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;

public class Arm {
    /*
    1 low
    2 mid
    3 high
    4 human station
    */

    // Objects
    Dashboard dash;
    ColorSensorV3 colorSensor;
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
    final double ARM_ENCODER_OFFSET = 155.390625;
    final double INTAKE_ENCODER_OFFSET = 142.470703125;

    // Constants
    final double ARM_LENGTH = 0.8218;
    final double ARM_PIVOT_HEIGHT = 0.9836;
    final double HAND_ROTATIONAL_SPEED = 0.28;
    final double ARM_MOTOR_SPEED = 0.3;
    final double INTAKE_ARM_MOTOR_SPEED = 0.3;
    final double INTAKE_ARM_SLOW_SPEED = 0.05;
    final double ARM_DEPLOY_SPEED = 1;
    final double ARM_SLOW_SPEED = 0.08;

    final double HIGH_CONE = 219.19921875; //DONE
    final double HIGH_CUBE = 216.771484375; //DONE
    final double MID_CONE = 204.697265625; //DONE
    final double MID_CUBE = 229.833984375; //DONE
    final double HYBRID = 325; //DONE
    final double DOCK = 10; //DONE
    final double IHIGH_CONE = 223.154296875; //DONE
    final double IHIGH_CUBE = 251.3671875; //DONE
    final double IMID_CONE = 289.599609375; //DONE
    final double IMID_CUBE = 287.2265625; //DONE
    final double IHYBRID = 113; //DONE
    final double IDOCK = 330; //DONE

    public double[] holdAng = {0,0};
    // Other Variables
    int goalLevel = 0;
    int armEncoderBuffer = 0;
    double armEncoderValue = 0;
    int intakeEncoderBuffer = 0;
    double intakeEncoderValue = 0;
    boolean holdingCone = false;
    boolean holdingCube = false;

    Arm() {

        dash = new Dashboard();
        
        // Arm
        armMotor = new CANSparkMax(ARM_ID, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(ARM_2_ID, MotorType.kBrushless);
        armMotor.setInverted(false);
        armMotor2.follow(armMotor, true);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor2.setIdleMode(IdleMode.kBrake);

        // Intake
        colorSensor = new ColorSensorV3(Port.kOnboard);
        intakeArmMotor = new CANSparkMax(INTAKE_ARM_ID, MotorType.kBrushless);
        intakeArmMotor.setIdleMode(IdleMode.kBrake);
        leftWheels = new CANSparkMax(LEFT_HAND_ID, MotorType.kBrushless);
        rightWheels = new CANSparkMax(RIGHT_HAND_ID, MotorType.kBrushless);
        leftWheels.setInverted(false);
        rightWheels.setInverted(true);

        // Pnuematics
        pcm = new PneumaticsControlModule();
        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        onSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        offSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

        pcm.clearAllStickyFaults();

        // Encoders
        armEncoder = new CANCoder(ARM_ENCODER_ID);
        armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        armEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armEncoder.configSensorDirection(true);
        armEncoder.configMagnetOffset(-ARM_ENCODER_OFFSET);
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

    }
    
    double[] calculateArmLevel(int level){
        // check color
        checkColor();
        // get angles
        double armAngle = getArmEncoder();
        double intakeAngle = getIntakeEncoder();
        // switch over the level, 0 meaning we have no level and should not be moving and 1,2,3 for heights.
        // all handle cone/cube movement, returning the right values for each, returns the result of calcArmOutputs.
        // returns a double[] of values, arm then intake, 0 if were at the zone and a value otherwise to go in the right direction.
        switch(level){
            case 1:
                // hybrid
                return calculateArmOutputs(armAngle, intakeAngle, HYBRID, IHYBRID);
            case 2:
                // mid
                if(holdingCone & !holdingCube){
                    // cone
                    return calculateArmOutputs(armAngle, intakeAngle, MID_CONE, IMID_CONE);
                } else {
                    // cube
                    return calculateArmOutputs(armAngle, intakeAngle, MID_CUBE, IMID_CUBE);
                }
            case 3:
                // high
                if(holdingCone & !holdingCube){
                    // cone
                    return calculateArmOutputs(armAngle, intakeAngle, HIGH_CONE, IHIGH_CONE);
                } else {
                    // we have a cube
                    return calculateArmOutputs(armAngle, intakeAngle, HIGH_CUBE, IHIGH_CUBE);
                 }
            case 4:
                 //dock
                 return calculateArmOutputs(armAngle, intakeAngle, DOCK, IDOCK);
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
        if (iAngle < ih-1) {
            returningVal[1] = INTAKE_ARM_SLOW_SPEED;
        } else if (iAngle > ih+1) {
            returningVal[1] = -INTAKE_ARM_SLOW_SPEED;
        }
        //slow intake
        if (iAngle < ih-10) {
            returningVal[1] = INTAKE_ARM_MOTOR_SPEED;
        } else if (iAngle > ih+10) {
            returningVal[1] = -INTAKE_ARM_MOTOR_SPEED;
        }

        // same here
        //fast deploy
        if (aAngle < 150 & !(goalLevel == 4)) {
            if (aAngle < th - 1) {
                returningVal[0] = ARM_DEPLOY_SPEED;
            } else if (aAngle > th+1) {
                returningVal[0] = -ARM_DEPLOY_SPEED;
            }
            returningVal[1] = 0;
        } else {
            if (aAngle < th - 1) {
                returningVal[0] = ARM_SLOW_SPEED;
            } else if (aAngle > th+1) {
                returningVal[0] = -ARM_SLOW_SPEED;
            }
            if (aAngle < th - 5) {
                returningVal[0] = ARM_MOTOR_SPEED;
            } else if (aAngle > th+5) {
                returningVal[0] = -ARM_MOTOR_SPEED;
            }

            
        }
        // no break intake from hybrid/ pickup position
        if (aAngle > 290 & !(goalLevel == 1)) {
            returningVal[1] = 0;
        }

        return returningVal;
    }

    void grabObject(boolean cube){
        
        if (cube) {
            leftWheels.set(-HAND_ROTATIONAL_SPEED);
            rightWheels.set(-HAND_ROTATIONAL_SPEED);

        } else {
            offSolenoid.set(false);
            onSolenoid.set(true);
            leftWheels.set(-HAND_ROTATIONAL_SPEED);
            rightWheels.set(-HAND_ROTATIONAL_SPEED);
        }
    }

    void placeObject(boolean cube){
        checkColor();
        if(cube){
            // holding a cube
            leftWheels.set(HAND_ROTATIONAL_SPEED);
            rightWheels.set(HAND_ROTATIONAL_SPEED);
            
        } else {
            // move the pneumatic cone bits
            offSolenoid.set(true);
            onSolenoid.set(false);
        }
        
    }

    public double getArmEncoder() {

        if (armEncoderBuffer++ > 5) {
            armEncoderBuffer = 0;
            armEncoderValue = armEncoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(armEncoderValue, 0, 360);

    }

    public double getIntakeEncoder() {

        if (intakeEncoderBuffer++ > 5) {
            intakeEncoderBuffer = 0;
            intakeEncoderValue = intakeArmEncoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(intakeEncoderValue, 0, 360);

    }
    
    void checkColor() {
        if (colorSensor.getColor() == Color.kYellow) {
            holdingCone = true;
            holdingCube = false;
        } else if (colorSensor.getColor() == Color.kPurple) {
            holdingCube = true;
            holdingCone = false;
        } else {
            holdingCone = false;
            holdingCube = false;
        }
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