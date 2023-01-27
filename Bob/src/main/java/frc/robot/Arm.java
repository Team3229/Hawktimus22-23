package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.AbsoluteEncoder;
/*
    Needs to do:
    Go from docked to undocked position and vice versa
    Semi-auto orient with april tag
    ID 4 is blues human station
    ID 5 is reds
    orient to place at different levels. (Peferable auto and not by driver eyeballing it)
    total inputs for whichever controller or one if single driver
    One of the four abxy buttons:
    -align with visible april tag (Assuming there is one) If driver inputs anything during this it overrules the auto
    -prepare to place high
    -low
    -mid
Right trigger - move to place object
left trigger - retract placing object if started to place
    these triggers can base off of whatever height mode were in to not overextend or underretreat.
    Rely on driver to get it lined up for wherever the object is going, arm controller just sets the height.
    Left,right sticks are as normal for swerve im assuming
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

    int encoderBuffer = 0;
    double encoderValue = 0;
    
    // motor 1 is the big arm and 2 is the smaller arm
    CANSparkMax longArm;
    CANSparkMax shortArm;

    CANSparkMax leftHand;
    CANSparkMax rightHand;
    
    AbsoluteEncoder longArmEncoder;
    AbsoluteEncoder shortArmEncoder;

    RelativeEncoder leftHandEncoder;
    RelativeEncoder rightHandEncoder;

    double[] pidv1 = {0,0,0};
    double[] pidv2 = {0,0,0};

    double leftHandLastValue = 0;
    double rightHandLastValue = 0;

    boolean isGrabbing = false;

    PIDController pid1 = new PIDController(pidv1[0], pidv1[1], pidv1[2]);
    PIDController pid2 = new PIDController(pidv2[0], pidv2[1], pidv2[2]);

    Arm() {

        longArm = new CANSparkMax(longArmID, MotorType.kBrushless);
        shortArm = new CANSparkMax(shortArmID, MotorType.kBrushless);
        leftHand = new CANSparkMax(leftHandID, MotorType.kBrushless);
        rightHand = new CANSparkMax(rightHandID, MotorType.kBrushless);

        longArmEncoder = longArm.getAbsoluteEncoder(Type.kDutyCycle);
        shortArmEncoder = shortArm.getAbsoluteEncoder(Type.kDutyCycle);

        leftHandEncoder =leftHand.getEncoder();
        rightHandEncoder = rightHand.getEncoder();

        SmartDashboard.putNumber("ArmLevel",armLevel);
        SmartDashboard.putNumberArray("PIDArm1", pidv1);
        SmartDashboard.putNumberArray("PIDArm2", pidv2);

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
                pid1.setSetpoint(0);
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

    void placeObject(){
        switch(armLevel){
            case 2:
                //begin to place low
                break;
            case 3:
                // place mid
                break;
            case 4:
                // place high
                break;
            default:
                return;
        }
    }
    
    void liftArm(){
        // lift the arm incase we started moving to place but are not in the right orientation or something i guess
        
    }

    void pickupObject(){
        if(!isGrabbing){
            return;
        }
        switch(armLevel){
            case 5:
                //pickup from ground
                if(!closeHands(true)){
                    isGrabbing = false;
                }
                break;
            case 6:
                //human station
                closeHands(true);
                break;
            default:
                break;
        }
    }

    boolean closeHands(boolean cube){

        if (cube) {
            //picking up cube

            leftHand.set((leftHand.get() != 0) ? (leftHandEncoder.getPosition() != leftHandLastValue) ? -0.07 : 0 : 0);
            rightHand.set((rightHand.get() != 0) ? (rightHandEncoder.getPosition() != rightHandLastValue) ? -0.07 : 0 : 0);

            leftHandLastValue = leftHandEncoder.getPosition();
            rightHandLastValue = rightHandEncoder.getPosition();

            if(leftHand.get() == 0 | rightHand.get() == 0){
                return false;
            } else {
                return true;
            }

        } else {
            return false;
        }

}

    //for testing only, remove once done
    // PLEASE DON"T FORGET :)
    void grabPiece() {

        leftHand.set((leftHand.get() != 0) ? (leftHandEncoder.getPosition() != leftHandLastValue) ? -0.07 : 0 : 0);
        rightHand.set((rightHand.get() != 0) ? (rightHandEncoder.getPosition() != rightHandLastValue) ? -0.07 : 0 : 0);

        leftHandLastValue = leftHandEncoder.getPosition();
        rightHandLastValue = rightHandEncoder.getPosition();

    }

    void startGrab() {

        leftHand.set(-0.07);
        rightHand.set(-0.07);

        leftHandLastValue = leftHandEncoder.getPosition();
        rightHandLastValue = rightHandEncoder.getPosition();

    }

    void release() {

        leftHand.set(0.07);
        rightHand.set(0.07);

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