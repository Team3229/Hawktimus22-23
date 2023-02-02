package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/*
    Needs to do:
    Go from docked to undocked position and vice versa
    Semi-auto orient with april tag
    ID 4 is blue´s human station
    ID 5 is red´s
    orient to place at different levels. (Preferable auto and not by driver eyeballing it)
    total inputs for whichever controller or one if single driver
    One of the four a,b,x,y xbox buttons:
    -align with visible april tag (Assuming there is one) If driver inputs anything during this it overrules the auto
    -prepare to place high
    -low
    -mid
Right trigger - move to place object
left trigger - retract placing object if started to place
    these triggers can base off of whatever height mode were in to not overextend or underretreat.
    Rely on driver to get it lined up for wherever the object is going, arm controller just sets the height.
    Left, right sticks are as normal for swerve I´m assuming
    same for dpad for slow movement
    Leftover inputs if all are on one controller:
    Left and right bumpers
    If split for two controllers, one arm and one drive
    Drive controller only uses sticks and dpad
    Manip controller uses abxy buttons and triggers
    Things we could auto outside of auto mode:
    -lining up with april tag
    We almost already have this with our follow tag mode - as long as our buffer zone is good.
    -placing cubes/cones depending on how we do the controls
    
All switch statements need to be filled in once we know how to program the arm

 0 is stowed pos
 +angle is around the long way avoiding dead zone
 340 max
*/

public class Arm {

Utils utils = new Utils();

/*
0 docked
1 low
2 mid
3 high
4 ground
5 human station
*/
int armLevel = 0;

final int longArmID = 16;
final int shortArmID = 17;
final int leftHandID = 14;
final int rightHandID = 15;
final double cubeBuffer = 0.00000001;

int encoderBuffer = 0;
double encoderValue = 0;

// motor 1 is the big arm and 2 is the smaller arm/wristed intake
CANSparkMax longArm;
CANSparkMax shortArm;

CANSparkMax leftHand;
CANSparkMax rightHand;

Solenoid onSolenoid;
Solenoid offSolenoid;

Compressor compressor;

AbsoluteEncoder longArmEncoder;
AbsoluteEncoder shortArmEncoder;

RelativeEncoder leftHandEncoder;
RelativeEncoder rightHandEncoder;

double[] longPIDv = {0,0,0};
double[] shortPIDv = {0,0,0};

double leftHandLastValue = 0;
double rightHandLastValue = 0;


boolean isGrabbing = false;

PIDController longPID = new PIDController(longPIDv[0], longPIDv[1], longPIDv[2]);
PIDController shortPID = new PIDController(shortPIDv[0], shortPIDv[1], shortPIDv[2]);

Arm() {

    longArm = new CANSparkMax(longArmID, MotorType.kBrushless);
    shortArm = new CANSparkMax(shortArmID, MotorType.kBrushless);
    leftHand = new CANSparkMax(leftHandID, MotorType.kBrushless);
    rightHand = new CANSparkMax(rightHandID, MotorType.kBrushless);

    compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
    onSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    offSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

    longArmEncoder = longArm.getAbsoluteEncoder(Type.kDutyCycle);
    shortArmEncoder = shortArm.getAbsoluteEncoder(Type.kDutyCycle);

    leftHandEncoder = leftHand.getEncoder();
    rightHandEncoder = rightHand.getEncoder();

    SmartDashboard.putNumber("CurrentArmLevel",armLevel);
    SmartDashboard.putNumberArray("LongArmPID", longPIDv);
    SmartDashboard.putNumberArray("ShortArmPID", shortPIDv);

}

void setArmLevel(int level){
    if(armLevel == level){
        return;
    }
    armLevel = level;
    SmartDashboard.putNumber("ArmLevel",armLevel);
    switch(armLevel){
        case 0:
            // dock
            longPID.setSetpoint(0);
            shortPID.setSetpoint(340);
            break;
        case 1:
            // low
            break;
        case 2:
            // mid
            break;
        case 3:
            // high
            break;
        case 4:
            // ground
            break;
        case 5:
            // station
            break;
        default:
            // go to docked pos
            break;
    }
}

void placeObject(boolean gamePiece){
    
    if(gamePiece){
        // holding a cube
        leftHand.set(0.07);
        rightHand.set(0.07);
        
    } else {
        // move the cone bits
        offSolenoid.set(true);
        onSolenoid.set(false);
    }
    
}

void closeHands(boolean cube){

    if (cube) {
        //picking up cube

        leftHand.set((leftHand.get() != 0) ? (leftHandEncoder.getPosition() != leftHandLastValue) ? -0.07 : 0 : 0);
        rightHand.set((rightHand.get() != 0) ? (rightHandEncoder.getPosition() != rightHandLastValue) ? -0.07 : 0 : 0);

        leftHandLastValue = leftHandEncoder.getPosition();
        rightHandLastValue = rightHandEncoder.getPosition();

    } else {
        // picking up cone
        offSolenoid.set(false);
        onSolenoid.set(true);
    }
}

void checkHandMotors() {

    // Cube
    // if they have not moved and are moving, stop them to prevent burnout.
    leftHand.set((leftHand.get() != 0) ? (Math.abs(leftHandEncoder.getPosition()) != Math.abs(leftHandLastValue) + cubeBuffer) ? -0.07 : 0 : 0);
    rightHand.set((rightHand.get() != 0) ? (Math.abs(rightHandEncoder.getPosition()) != Math.abs(rightHandLastValue) + cubeBuffer) ? -0.07 : 0 : 0);

    leftHandLastValue = leftHandEncoder.getPosition();
    rightHandLastValue = rightHandEncoder.getPosition();

}

void startCubeGrab() {

    leftHand.set(-0.07);
    rightHand.set(-0.07);
    leftHandLastValue = leftHandEncoder.getPosition();
    rightHandLastValue = rightHandEncoder.getPosition();

}

void softStop() {

    if (leftHand.get() == 0.07 | rightHand.get() == 0.07) {

        leftHand.stopMotor();
        rightHand.stopMotor();

    } else if (leftHand.get() == 0 | rightHand.get() == 0) {

        leftHand.stopMotor();
        rightHand.stopMotor();

    }

}
}