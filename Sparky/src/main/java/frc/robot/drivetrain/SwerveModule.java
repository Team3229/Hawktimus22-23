//Otters: 3229 Programming SubTeam

package frc.robot.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import frc.robot.Utils;

public class SwerveModule {

    CANSparkMax driveMotor;
    CANSparkMax angleMotor;
    CANCoder canCoder;
    RelativeEncoder driveEncoder;
    RelativeEncoder angleEncoder;
    SparkMaxPIDController anglePIDController;
    SparkMaxPIDController drivePIDController;
    SwerveModuleState moduleState;

   
    /**Location of the Swerve Module realtive to robot center*/
    Translation2d location;
    int encoderBuffer = 0;
    /**Current postition of Swerve Module encoder, modified by magnet offset*/
    double encoderValue = 0;
    /**Current RPM of Swerve Module wheel*/
    double driveRPM = 0;

    double encoderOffset = 0;
    
    // RPM/sec
    final double angleMaxAccel = 1;
    // RPM
    final double angleMaxVel = 99999;

    final double wheelRadius = 0.0508;

    public SwerveModule(int driveID, int angleID, int encoderID, double[] anglePID, double[] drivePID, double X, double Y, boolean invertMotor) {

        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);

        canCoder = new CANCoder(encoderID);
        canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
        driveEncoder = driveMotor.getEncoder();
        angleMotor.setInverted(false);
        angleEncoder = angleMotor.getEncoder();

        anglePIDController = angleMotor.getPIDController();
        drivePIDController = driveMotor.getPIDController();
        anglePIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        anglePIDController.setSmartMotionMaxAccel(angleMaxAccel, 0);
        anglePIDController.setSmartMotionMaxVelocity(angleMaxVel, 0);
        
        location = new Translation2d(X/2, Y/2);
    
        driveMotor.setInverted(invertMotor);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setOpenLoopRampRate(0.35);

        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    }

    void configEncoder(double oofset) {

        canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        canCoder.configMagnetOffset(oofset);

        angleEncoder.setPositionConversionFactor(360/12.8);
        angleEncoder.setPosition(getCANCoder());

    }

    void configPID(double[] anglePID, double[] drivePIDFF) {
        anglePIDController.setP(anglePID[0]);
        anglePIDController.setI(anglePID[1]);
        anglePIDController.setD(anglePID[2]);
        drivePIDController.setP(drivePIDFF[0]);
        drivePIDController.setI(drivePIDFF[1]);
        drivePIDController.setD(drivePIDFF[2]);
        anglePIDController.setFF(drivePIDFF[3]);
        anglePIDController.setPositionPIDWrappingMinInput(0);
        anglePIDController.setPositionPIDWrappingMaxInput(360);
        anglePIDController.setPositionPIDWrappingEnabled(true);
    }

    void setState(SwerveModuleState moduleSta) {

        moduleState = SwerveModuleState.optimize(moduleSta, Rotation2d.fromDegrees(MathUtil.inputModulus(angleEncoder.getPosition(), 0, 360)));
        
        // drivePIDController.setReference(Utils.convertMpsToRpm(moduleState.speedMetersPerSecond, wheelRadius), ControlType.kVelocity);
        driveMotor.set(moduleState.speedMetersPerSecond);
        anglePIDController.setReference(moduleState.angle.getDegrees(), ControlType.kPosition);

    }

    double getCANCoder() {

        encoderValue = canCoder.getAbsolutePosition();

        return MathUtil.inputModulus(encoderValue, 0, 360);

    }

    void stop() {

        angleMotor.stopMotor();
        driveMotor.stopMotor();

    }
    
}