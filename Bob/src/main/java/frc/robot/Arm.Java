package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
*/

public class Arm {

    Utils utils = new Utils();
    
    // 0 Docked, 1 Undocked, 2,3,4 Low,mid,high 5 ground, 6, humanstation Square
    int armLevel = 0;
    final int motor1ID = 13;
    final int motor2ID = 14;
    final int encoder1ID = 13;
    final int encoder2ID = 14;

    int encoderBuffer = 0;
    double encoderValue = 0;
    
    // motor 1 is the big arm and 2 is the smaller arm
    CANSparkMax motor1;
    CANSparkMax motor2;
    
    CANCoder encoder1;
    CANCoder encoder2;

    double[] pidv1 = {0,0,0};
    double[] pidv2 = {0,0,0};

    PIDController pid1 = new PIDController(pidv1[0], pidv1[1], pidv1[2]);
    PIDController pid2 = new PIDController(pidv2[0], pidv2[1], pidv2[2]);

    Arm() {

        motor1 = new CANSparkMax(motor1ID, MotorType.kBrushless);
        motor2 = new CANSparkMax(motor2ID, MotorType.kBrushless);

        encoder1 = new CANCoder(encoder1ID);
        encoder2 = new CANCoder(encoder2ID);

        SmartDashboard.putNumber("ArmLevel",armLevel);
        SmartDashboard.putNumberArray("PIDArm1", pidv1);
        SmartDashboard.putNumberArray("PIDArm2", pidv2);

    }

    double GetAbsoluteEncoder(CANCoder encoder) {
 
        if (encoderBuffer++ > 5) {
            encoderBuffer = 0;
            encoderValue = encoder.getAbsolutePosition();
        }

        return utils.convertAngle(encoderValue);

    }

    void setArmLevel(int level){
        if(armLevel == level){
            return;
        }
        armLevel = level;
        SmartDashboard.putNumber("ArmLevel",armLevel);
        switch(armLevel){
            case 1:
                // Undock
                break;
            case 2:
                // Low
                break;
            case 3:
                // Mid
                break;
            case 4:
                // High
                break;
            case 5:
                // Ground
                break;
            case 6:
                // Human Station Height
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
        switch(armLevel){
            case 5:
                //pickup from ground
                break;
            case 6:
                //human station
                break;
            default:
                break;
        }
    }

}