package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Intake class for controlling the intake */
public class Intake {
    
    CANSparkMax intakeMotor;
    CANSparkMax intakeFollowerMotor;

    DoubleSolenoid solenoid;
    PneumaticsControlModule pcm;
    Compressor compressor;

    // IDs
    private static final int LEFT_ID = 14;
    private static final int RIGHT_ID = 15;
    private static final int ON_ID = 5;
    private static final int OFF_ID = 4;

    private static final double IN_HAND_ROTATIONAL_SPEED = -0.2;
    private static final double OUT_HAND_ROTATIONAL_SPEED = 0.1;

    /**Initilizes all the intakes motors, pneumatics, and compressor */
    Intake() {
        intakeMotor = new CANSparkMax(LEFT_ID, MotorType.kBrushless);
        intakeFollowerMotor = new CANSparkMax(RIGHT_ID, MotorType.kBrushless);
        intakeMotor.setInverted(false);
        intakeFollowerMotor.setInverted(true);

        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ON_ID, OFF_ID);

        pcm = new PneumaticsControlModule();
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);

        pcm.clearAllStickyFaults();

        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        intakeFollowerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    }

    /**
     * Grabs an object
     * @param cube (boolean) Is the object we are grabbing a cube
     */
    void grabObject(boolean cube){
        
        if (!cube) {
            solenoid.set(Value.kForward);
            intakeMotor.set(IN_HAND_ROTATIONAL_SPEED*0.6);
            intakeFollowerMotor.set(IN_HAND_ROTATIONAL_SPEED*0.6);
        } else {
            solenoid.set(Value.kReverse);
            intakeMotor.set(IN_HAND_ROTATIONAL_SPEED);
            intakeFollowerMotor.set(IN_HAND_ROTATIONAL_SPEED);
        }
    }

    /**
     * Place the currently held object
     * @param cube (boolean) Is the currently held object a cube
     */
    void placeObject(boolean cube){

        if(!cube){
            solenoid.set(Value.kForward);
            intakeMotor.set(OUT_HAND_ROTATIONAL_SPEED*2.5);
            intakeFollowerMotor.set(OUT_HAND_ROTATIONAL_SPEED*2.5);
        } else {
            solenoid.set(Value.kReverse);
            intakeMotor.set(OUT_HAND_ROTATIONAL_SPEED);
            intakeFollowerMotor.set(OUT_HAND_ROTATIONAL_SPEED);
        }
        
    }

    /** Stops the intake motors */
    void stopMotors() {
        intakeMotor.stopMotor();
        intakeFollowerMotor.stopMotor();
    }

}
