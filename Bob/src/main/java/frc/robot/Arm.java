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


    Dashboard dash;

    /*
    1 low
    2 mid
    3 high
    4 human station
    */

    ColorSensorV3 colorSensor;

    final int longArmID = 16;
    final int longArmID2 = 17;

    final int intakeArmID = 18;
    final int leftHandID = 14;
    final int rightHandID = 15;

    final double cubeDeadZone = 0.1;
    final double armLength = 0.8218;
    final double armPivotHeight = 0.9836;

    final double cubeSpeed = 0.14;
    final double armSpeed = 0.01;
    final double backArmSpeed = 0.001;
    final double intakeArmSpeed = 0.001;

    //50 in
    final double highCone = 11.277;
    //39.5 in
    final double highCube = 0.777;
    //38 in
    final double midCone = -0.723;
    //27.5 in
    final double midCube = -11.223;
    //4 in
    final double hybrid = -34.723;

    int currentAttemptedLevel = 0;
    int encoderBuffer1 = 0;
    double encoderValue1 = 0;
    int encoderBuffer2 = 0;
    double encoderValue2 = 0;

    boolean holdingCone = false;
    boolean holdingCube = false;
    
    double leftWheelsLastValue = 0;
    double rightWheelsLastValue = 0;

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

    Arm() {

        dash = new Dashboard();

        colorSensor = new ColorSensorV3(Port.kOnboard);

        armMotor = new CANSparkMax(longArmID, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(longArmID2, MotorType.kBrushless);
        armMotor.setInverted(false);
        armMotor2.follow(armMotor, true);

        intakeArmMotor = new CANSparkMax(intakeArmID, MotorType.kBrushless);
        leftWheels = new CANSparkMax(leftHandID, MotorType.kBrushless);
        rightWheels = new CANSparkMax(rightHandID, MotorType.kBrushless);

        leftWheels.setInverted(true);
        rightWheels.setInverted(false);

        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        onSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        offSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

        armEncoder = new CANCoder(19);
        armEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        armEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        armEncoder.configSensorDirection(true);
        armEncoder.configMagnetOffset(-208.765625);
        intakeArmEncoder = new CANCoder(20);

        leftWheelsEncoder = leftWheels.getEncoder();
        rightWheelsEncoder = rightWheels.getEncoder();

    }
    /*
    intake needs to stay level at all times. 249.5-relative angle
    */
    double[] calculateArmLevel(int level){

        checkColor();
        dash.putNumber("ArmLevel", level);
        double armAngle = getArmEncoder();
        double intakeAngle = getIntakeEncoder();
        switch(level){
            case 1:
                // hybrid
                return new double[] {
                    ((armAngle > armGetAngleFromHeight(hybrid)+1) ? -armSpeed : ((armAngle < armGetAngleFromHeight(hybrid)-1) ? armSpeed : 0)),
                    ((intakeAngle > 249.5-armGetAngleFromHeight(hybrid)+1) ? -intakeArmSpeed : ((intakeAngle < 249.5-armGetAngleFromHeight(hybrid)-1) ? intakeArmSpeed : 0))
                };
            case 2:
                // mid
                if(holdingCone & !holdingCube){
                    // we have a cone
                    return new double[] {
                        ((armAngle > armGetAngleFromHeight(midCone)+1) ? -armSpeed : ((armAngle < armGetAngleFromHeight(midCone)-1) ? armSpeed : 0)),
                        ((intakeAngle > 249.5-armGetAngleFromHeight(midCone)+1) ? -intakeArmSpeed : ((intakeAngle < 249.5-armGetAngleFromHeight(midCone)-1) ? intakeArmSpeed : 0))
                    };
                } else {
                    // we have a cube
                    return new double[]{
                        ((armAngle > armGetAngleFromHeight(midCube)+1) ? -armSpeed : ((armAngle < armGetAngleFromHeight(midCube)-1) ? armSpeed : 0)),
                        ((intakeAngle > 249.5-armGetAngleFromHeight(midCube)+1) ? -intakeArmSpeed : ((intakeAngle < 249.5-armGetAngleFromHeight(midCube)-1) ? intakeArmSpeed : 0))
                    };
                }
            case 3:
                // high
                if(holdingCone & !holdingCube){
                    // we have a cone
                    return new double[]{
                        ((armAngle > armGetAngleFromHeight(highCone)+1) ? -armSpeed : ((armAngle < armGetAngleFromHeight(highCone)-1) ? armSpeed : 0)),
                        ((intakeAngle > 249.5-armGetAngleFromHeight(highCone)+1) ? -intakeArmSpeed : ((intakeAngle < 249.5-armGetAngleFromHeight(highCone)-1) ? intakeArmSpeed : 0))  
                    };
                } else {
                    // we have a cube
                    return new double[]{
                        ((armAngle > armGetAngleFromHeight(highCube)+1) ? -armSpeed : ((armAngle < armGetAngleFromHeight(highCube)-1) ? armSpeed : 0)),
                        ((intakeAngle > 249.5-armGetAngleFromHeight(highCube)+1) ? -intakeArmSpeed : ((intakeAngle < 249.5-armGetAngleFromHeight(highCube)-1) ? intakeArmSpeed : 0))
                    };
                 }
            case 4:
                // station
                return new double[] {};
            default:
                return new double[] {};
        }
    }

    void placeObject(boolean cube){
        checkColor();
        if(cube){
            // holding a cube
            leftWheels.set(cubeSpeed);
            rightWheels.set(cubeSpeed);
            
        } else {
            // move the pneumatic cone bits
            offSolenoid.set(true);
            onSolenoid.set(false);
        }
        
    }

    void closeHands(boolean cube){
        
        if (cube) {
            
            leftWheels.set(-cubeSpeed);
            rightWheels.set(-cubeSpeed);
            leftWheelsLastValue = leftWheelsEncoder.getPosition();
            rightWheelsLastValue = rightWheelsEncoder.getPosition();

        } else {
            offSolenoid.set(false);
            onSolenoid.set(true);
        }
    }

    private double getArmEncoder() {

        if (encoderBuffer1++ > 5) {
            encoderBuffer1 = 0;
            encoderValue1 = armEncoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(encoderValue1, 0, 360);

    }

    private double getIntakeEncoder() {

        if (encoderBuffer2++ > 5) {
            encoderBuffer2 = 0;
            encoderValue2 = intakeArmEncoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(encoderValue2, 0, 360);

    }

    void checkHandMotors() {

        // For Cube
        // if they have not moved and are moving, stop them to prevent burnout.
        leftWheels.set((leftWheels.get() < 0) ? (Math.abs(leftWheelsEncoder.getPosition()) != Math.abs(leftWheelsLastValue) + cubeDeadZone) ? -cubeSpeed : 0 : leftWheels.get());
        rightWheels.set((rightWheels.get() < 0) ? (Math.abs(rightWheelsEncoder.getPosition()) != Math.abs(rightWheelsLastValue) + cubeDeadZone) ? -cubeSpeed : 0 : rightWheels.get());

        leftWheelsLastValue = leftWheelsEncoder.getPosition();
        rightWheelsLastValue = rightWheelsEncoder.getPosition();

    }

    double armGetAngleFromHeight(double height){
        return (Math.acos(height/32));
    }
    
    void softStop() {

        if (leftWheels.get() == cubeSpeed | rightWheels.get() == cubeSpeed) {

            leftWheels.stopMotor();
            rightWheels.stopMotor();

        } else if (leftWheels.get() == 0 | rightWheels.get() == 0) {

            leftWheels.stopMotor();
            rightWheels.stopMotor();
        }
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
        currentAttemptedLevel = level;
    }

    void runArm() {
        if (currentAttemptedLevel != 0) {
        double[] armResults = calculateArmLevel(currentAttemptedLevel);
        if (armResults[0] != 0) {
            armMotor.set(armResults[0]);
        } else {
            armMotor.stopMotor();
        }
        if (armResults[1] != 0) {
            intakeArmMotor.set(armResults[1]);
        } else {
            intakeArmMotor.stopMotor();
        }
        if(armResults[1] == 0 & armResults[0] == 0){
            currentAttemptedLevel = 0;
        }
    }
    }

}