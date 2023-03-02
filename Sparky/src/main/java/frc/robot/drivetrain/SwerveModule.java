//Otters: 3229 Programming SubTeam

package frc.robot.drivetrain;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    CANSparkMax driveMotor;
    CANSparkMax angleMotor;
    CANCoder encoder;
    RelativeEncoder driveEncoder;
    RelativeEncoder angleEncoder;
    PIDController anglePIDController;
    PIDController drivePIDController;
    SwerveModuleState moduleState;

   
    /**Location of the Swerve Module realtive to robot center*/
    Translation2d location;
    int encoderBuffer = 0;
    /**Current postition of Swerve Module encoder, modified by magnet offset*/
    double encoderValue = 0;
    /**Current RPM of Swerve Module wheel*/
    double driveRPM = 0;

    double encoderOffset = 0;

    final double anglePosTolerance = 1;
    final double angleVelTolerance = 1;

    public SwerveModule(int driveID, int angleID, int encoderID, double[] anglePID, double[] drivePID, double X, double Y, boolean invertMotor) {

        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);

        encoder = new CANCoder(encoderID);
        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100);
        driveEncoder = driveMotor.getEncoder();
        angleMotor.setInverted(false);

        anglePIDController = new PIDController(anglePID[0], anglePID[1], anglePID[2]);
        anglePIDController.setTolerance(anglePosTolerance, angleVelTolerance);
        
        location = new Translation2d(X/2, Y/2);
    
        driveMotor.setInverted(invertMotor);

        driveMotor.setOpenLoopRampRate(0.25);

        driveMotor.setIdleMode(IdleMode.kBrake);

    }

    void configEncoder(double oofset) {

        encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        
        encoder.configMagnetOffset(oofset);

    }
    
    void configPID(double[] anglePID, double[] drivePID) {
        
        anglePIDController.setPID(anglePID[0],anglePID[1],anglePID[2]);
        // drivePIDController.setP(drivePID[0]);
        // drivePIDController.setI(drivePID[1]);
        // drivePIDController.setD(drivePID[2]);
        //drivePIDController.setPID(drivePID[0],drivePID[1],drivePID[2])
        anglePIDController.enableContinuousInput(0, 360);
        anglePIDController.setTolerance(anglePosTolerance, angleVelTolerance);

    }

    void setState(SwerveModuleState moduleSta) {

        driveRPM = driveEncoder.getVelocity();

        moduleState = SwerveModuleState.optimize(moduleSta, Rotation2d.fromDegrees(getEncoder()));
        
        // drivePIDController.setReference(utils.mpsToRPM(moduleState.speedMetersPerSecond), ControlType.kVelocity);
        driveMotor.set(moduleState.speedMetersPerSecond);
        // angleMotor.set(anglePIDController.calculate(GetAbsoluteEncoder(), moduleState.angle.getDegrees()));
        angleMotor.set(anglePIDController.calculate(getEncoder(), moduleState.angle.getDegrees()));
    
            
    }

    double getEncoder() {

        if (encoderBuffer++ > 5) {
            encoderBuffer = 0;
            encoderValue = encoder.getAbsolutePosition();
        }

        return MathUtil.inputModulus(encoderValue, 0, 360);

    }

    void stop() {

        angleMotor.stopMotor();
        // drivePIDController.setReference(0, ControlType.kVelocity);
        driveMotor.stopMotor();

    }
    
}