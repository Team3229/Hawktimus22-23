package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class Arm {

    Utils utils = new Utils();

    /*
    0 docked
    1 low
    2 mid
    3 high
    4 human station
    */
    int armLevel = 0;

    ColorSensorV3 colorSensor;

    final int longArmID = 16;
    final int intakeArmID = 17;
    final int leftHandID = 14;
    final int rightHandID = 15;
    final double cubeDeadZone = 0.1;
    final double armLength = 0.8218;
    final double armPivotHeight = 0.9836;

    boolean holdingCone = false;
    boolean holdingCube = false;

    private static double cubeSpeed = 0.14;

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

    int encoderBuffer = 0;
    double encoderValue = 0;

    CANSparkMax armMotor;
    CANSparkMax intakeArmMotor;

    CANSparkMax leftWheels;
    CANSparkMax rightWheels;

    Solenoid onSolenoid;
    Solenoid offSolenoid;

    Compressor compressor;

    AbsoluteEncoder armEncoder;
    AbsoluteEncoder intakeArmEncoder;

    RelativeEncoder leftWheelsEncoder;
    RelativeEncoder rightWheelsEncoder;

    double[] armPIDv = {0,0,0};
    double[] intakeArmPIDv = {0,0,0};

    double leftWheelsLastValue = 0;
    double rightWheelsLastValue = 0;

    PIDController armPID = new PIDController(armPIDv[0], armPIDv[1], armPIDv[2]);
    PIDController intakeArmPID = new PIDController(intakeArmPIDv[0], intakeArmPIDv[1], intakeArmPIDv[2]);

    Arm() {

        colorSensor = new ColorSensorV3(Port.kOnboard);

        armMotor = new CANSparkMax(longArmID, MotorType.kBrushless);
        intakeArmMotor = new CANSparkMax(intakeArmID, MotorType.kBrushless);
        leftWheels = new CANSparkMax(leftHandID, MotorType.kBrushless);
        rightWheels = new CANSparkMax(rightHandID, MotorType.kBrushless);

        leftWheels.setInverted(true);
        rightWheels.setInverted(false);

        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        onSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        offSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

        armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
        intakeArmEncoder = intakeArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

        leftWheelsEncoder = leftWheels.getEncoder();
        rightWheelsEncoder = rightWheels.getEncoder();

        SmartDashboard.putNumber("CurrentArmLevel",armLevel);
        SmartDashboard.putNumberArray("LongArmPID", armPIDv);
        SmartDashboard.putNumberArray("ShortArmPID", intakeArmPIDv);

    }

    void setArmLevel(int level){

        checkColor();

        if(armLevel == level){
            return;
        }
        armLevel = level;
        SmartDashboard.putNumber("ArmLevel",armLevel);
        switch(armLevel){
            case 0:
                // dock
                armPID.setSetpoint(340);
                intakeArmPID.setSetpoint(45);
                break;
            case 1:
                // hybrid
                armPID.setSetpoint(armGetAngleFromHeight(hybrid));
                break;
            case 2:
                // mid
                if(holdingCone & !holdingCube){
                    // we have a cone
                    armPID.setSetpoint(armGetAngleFromHeight(midCone));
                } else if(holdingCube & !holdingCone){
                    // we have a cube
                    armPID.setSetpoint(armGetAngleFromHeight(midCube));
                } else {
                    // driver ur an idiot
                    DriverStation.reportError("DRIVERS AN IDIOT LOL", null);
                }
                break;
            case 3:
                // high
                if(holdingCone & !holdingCube){
                    // we have a cone
                    armPID.setSetpoint(armGetAngleFromHeight(highCone));
                } else if(holdingCube & !holdingCone){
                    // we have a cube
                    armPID.setSetpoint(armGetAngleFromHeight(highCube));
                } else {
                    // driver ur an idiot
                    DriverStation.reportError("DRIVERS AN IDIOT LOL", null);
                }
                break;
            case 4:
                // station
                break;
            default:
                // go to docked pos
                break;
        }
    }

    void placeObject(boolean cube){
        checkColor();
        if(cube){
            // holding a cube
            leftWheels.set(cubeSpeed);
            rightWheels.set(cubeSpeed);
            
        } else {
            // move the cone bits
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

    void checkHandMotors() {

        // For Cube
        // if they have not moved and are moving, stop them to prevent burnout.
        leftWheels.set((leftWheels.get() < 0) ? (Math.abs(leftWheelsEncoder.getPosition()) != Math.abs(leftWheelsLastValue) + cubeDeadZone) ? -cubeSpeed : 0 : leftWheels.get());
        rightWheels.set((rightWheels.get() < 0) ? (Math.abs(rightWheelsEncoder.getPosition()) != Math.abs(rightWheelsLastValue) + cubeDeadZone) ? -cubeSpeed : 0 : rightWheels.get());

        leftWheelsLastValue = leftWheelsEncoder.getPosition();
        rightWheelsLastValue = rightWheelsEncoder.getPosition();

    }

    double armGetAngleFromHeight(double height){
        return Math.acos(height/32);
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

}
