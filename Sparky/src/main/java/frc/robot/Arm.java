//Otters: 3229 Programming Sub-Team

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
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
    final double ARM_ENCODER_OFFSET = -208.765625;
    final double INTAKE_ENCODER_OFFSET = 0;

    // Constants
    final double HAND_DEAD_ZONE = 0.1;
    final double ARM_LENGTH = 0.8218;
    final double ARM_PIVOT_HEIGHT = 0.9836;
    final double HAND_ROTATIONAL_SPEED = 0.14;
    final double ARM_MOTOR_SPEED = 0.01;
    final double SLOW_ARM_MOTOR_SPEED = 0.001;
    final double INTAKE_ARM_MOTOR_SPEED = 0.001;
    final double HIGH_CONE = 11.277; //50 in
    final double HIGH_CUBE = 0.777; //39.5 in
    final double MID_CONE = -0.723; //38 in
    final double MID_CUBE = -11.223; //27.5 in
    final double HYBRID = -34.723; //4 in

    // Other Variables
    int goalLevel = 0;
    int armEncoderBuffer = 0;
    double armEncoderValue = 0;
    int intakeEncoderBuffer = 0;
    double intakeEncoderValue = 0;
    boolean holdingCone = false;
    boolean holdingCube = false;
    double leftWheelsLastValue = 0;
    double rightWheelsLastValue = 0;

    Arm() {

        dash = new Dashboard();
        
        // Arm
        armMotor = new CANSparkMax(ARM_ID, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(ARM_2_ID, MotorType.kBrushless);
        armMotor.setInverted(false);
        armMotor2.follow(armMotor, true);

        // Intake
        colorSensor = new ColorSensorV3(Port.kOnboard);
        intakeArmMotor = new CANSparkMax(INTAKE_ARM_ID, MotorType.kBrushless);
        leftWheels = new CANSparkMax(LEFT_HAND_ID, MotorType.kBrushless);
        rightWheels = new CANSparkMax(RIGHT_HAND_ID, MotorType.kBrushless);
        leftWheels.setInverted(true);
        rightWheels.setInverted(false);

        // Pnuematics
        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        onSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        offSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

        // Encoders
        armEncoder = new CANCoder(ARM_ENCODER_ID);
        armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        armEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armEncoder.configSensorDirection(true);
        armEncoder.configMagnetOffset(ARM_ENCODER_OFFSET);

        intakeArmEncoder = new CANCoder(INTAKE_ENCODER_ID);
        intakeArmEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        intakeArmEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        intakeArmEncoder.configSensorDirection(true);
        intakeArmEncoder.configMagnetOffset(INTAKE_ENCODER_OFFSET);

        leftWheelsEncoder = leftWheels.getEncoder();
        rightWheelsEncoder = rightWheels.getEncoder();

    }
    /*
    intake needs to stay level at all times except docked. 249.5-relative angle
    */
    double[] calculateArmLevel(int level){

        checkColor();
        double armAngle = getArmEncoder();
        double intakeAngle = getIntakeEncoder();

        switch(level){
            case 1:
                // hybrid
                return calculateArmOutputs(armAngle, intakeAngle, HYBRID);
            case 2:
                // mid
                if(holdingCone & !holdingCube){
                    // cone
                    return calculateArmOutputs(armAngle, intakeAngle, MID_CONE);
                } else {
                    // cube
                    return calculateArmOutputs(armAngle, intakeAngle, MID_CUBE);
                }
            case 3:
                // high
                if(holdingCone & !holdingCube){
                    // cone
                    return calculateArmOutputs(armAngle, intakeAngle, HIGH_CONE);
                } else {
                    // we have a cube
                    return calculateArmOutputs(armAngle, intakeAngle, HIGH_CUBE);
                 }
            case 4:
                // station
                return new double[] {};
            default:
                return new double[] {};
        }
    }

    double[] calculateArmOutputs(double aAngle, double iAngle, double height) {
        return new double[]{
            ((aAngle > (Math.acos(height/32))+1) ? -ARM_MOTOR_SPEED : ((aAngle < (Math.acos(height/32))-1) ? ARM_MOTOR_SPEED : 0)),
            ((iAngle > 249.5-(Math.acos(height/32))+1) ? -INTAKE_ARM_MOTOR_SPEED : ((iAngle < 249.5-(Math.acos(height/32))-1) ? INTAKE_ARM_MOTOR_SPEED : 0))
        };
    }

    void grabObject(boolean cube){
        
        if (cube) {
            leftWheels.set(-HAND_ROTATIONAL_SPEED);
            rightWheels.set(-HAND_ROTATIONAL_SPEED);
            leftWheelsLastValue = leftWheelsEncoder.getPosition();
            rightWheelsLastValue = rightWheelsEncoder.getPosition();

        } else {
            offSolenoid.set(false);
            onSolenoid.set(true);
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

    void checkIntakeMotors() {

        leftWheels.set((leftWheels.get() < 0) ? (Math.abs(leftWheelsEncoder.getPosition()) != Math.abs(leftWheelsLastValue) + HAND_DEAD_ZONE) ? -HAND_ROTATIONAL_SPEED : 0 : leftWheels.get());rightWheels.set((rightWheels.get() < 0) ? (Math.abs(rightWheelsEncoder.getPosition()) != Math.abs(rightWheelsLastValue) + HAND_DEAD_ZONE) ? -HAND_ROTATIONAL_SPEED : 0 : rightWheels.get());

        leftWheelsLastValue = leftWheelsEncoder.getPosition();
        rightWheelsLastValue = rightWheelsEncoder.getPosition();

    }

    void softStop() {

        if (leftWheels.get() == HAND_ROTATIONAL_SPEED | rightWheels.get() == HAND_ROTATIONAL_SPEED) {

            leftWheels.stopMotor();
            rightWheels.stopMotor();

        } else if (leftWheels.get() == 0 | rightWheels.get() == 0) {

            leftWheels.stopMotor();
            rightWheels.stopMotor();
        }
    }

    private double getArmEncoder() {

        if (armEncoderBuffer++ > 5) {
            armEncoderBuffer = 0;
            armEncoderValue = armEncoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(armEncoderValue, 0, 360);

    }

    private double getIntakeEncoder() {

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

    void setCurrentLevel(int level){
        goalLevel = level;
    }

    void runArm() {
        // Arm
        if (goalLevel != 0) {
        double[] armResults = calculateArmLevel(goalLevel);
        if (armResults[0] != 0) {
            armMotor.set(armResults[0]);
        } else {
            armMotor.stopMotor();
        }

        // Intake
        if (armResults[1] != 0) {
            intakeArmMotor.set(armResults[1]);
        } else {
            intakeArmMotor.stopMotor();
        }
        if(armResults[1] == 0 & armResults[0] == 0){
            goalLevel = 0;
        }
    }
    }

}