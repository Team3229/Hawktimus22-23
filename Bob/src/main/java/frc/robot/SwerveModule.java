// Otter: Nathan Manhardt (3229 Software Captain)
// Co-Otters: 3229 Software Team

package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;

public class SwerveModule {

    CANSparkMax driveMotor;
    CANSparkMax angleMotor;
    CANCoder encoder;
    RelativeEncoder driveEncoder;
    RelativeEncoder angleEncoder;
    SparkMaxPIDController anglePIDController;
    SparkMaxPIDController drivePIDController;
    Utils utils;
    SwerveModuleState moduleState;

    /**Location of the Swerve Module realtive to robot center*/
    Translation2d location;
    int encoderBuffer = 0;
    /**Current postition of Swerve Module encoder, modified by magnet offset*/
    double encoderValue = 0;
    /**Current RPM of Swerve Module wheel*/
    double driveRPM = 0;

    double encoderOffset = 0;
    double customOutput = 0;
    double customOutput2 = 0;

    public SwerveModule(int driveID, int angleID, int encoderID, double[] anglePID, double[] drivePID, double X, double Y, boolean invertMotor) {

        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);

        encoder = new CANCoder(encoderID);
        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();
        angleEncoder.setPositionConversionFactor(360);
        drivePIDController = driveMotor.getPIDController();
        angleMotor.setInverted(false);

        anglePIDController = angleMotor.getPIDController();
        drivePIDController.setP(drivePID[0]);
        drivePIDController.setI(drivePID[1]);
        drivePIDController.setD(drivePID[2]);

        location = new Translation2d(X/2, Y/2);

        utils = new Utils();
    
        driveMotor.setInverted(invertMotor);

        driveMotor.setOpenLoopRampRate(0.25);

        driveMotor.setIdleMode(IdleMode.kBrake);

    }

    void ConfigEncoder(double oofset) {

        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        encoder.configMagnetOffset(oofset);

        angleEncoder.setPosition(encoder.getAbsolutePosition());

    }

    void ConfigPID(double[] anglePID, double[] drivePID) {
        
        anglePIDController.setP(anglePID[0]);
        anglePIDController.setI(anglePID[1]);
        anglePIDController.setD(anglePID[2]);
        drivePIDController.setP(drivePID[0]);
        drivePIDController.setI(drivePID[1]);
        drivePIDController.setD(drivePID[2]);
        anglePIDController.setPositionPIDWrappingEnabled(true);
        anglePIDController.setPositionPIDWrappingMinInput(0);
        anglePIDController.setPositionPIDWrappingMaxInput(360);
        angleMotor.burnFlash();
        driveMotor.burnFlash();

    }

    void SetState(SwerveModuleState moduleSta) {

        driveRPM = driveEncoder.getVelocity();

        moduleState = SwerveModuleState.optimize(moduleSta, Rotation2d.fromDegrees(angleEncoder.getPosition()));
        
        // drivePIDController.setReference(utils.mpsToRPM(moduleState.speedMetersPerSecond), ControlType.kVelocity);
        driveMotor.set(moduleState.speedMetersPerSecond);
        // angleMotor.set(anglePIDController.calculate(GetAbsoluteEncoder(), moduleState.angle.getDegrees())/100);
        anglePIDController.setReference(MathUtil.inputModulus(moduleState.angle.getDegrees(), 0, 360), ControlType.kPosition);
    
            
    }

    double GetEncoder() {

        return angleEncoder.getPosition();

    }

    void Stop() {

        angleMotor.stopMotor();
        drivePIDController.setReference(0, ControlType.kVelocity);
        driveMotor.stopMotor();

    }
    
}