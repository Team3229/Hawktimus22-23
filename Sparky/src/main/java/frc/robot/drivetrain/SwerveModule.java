package frc.robot.drivetrain;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    
    // Motors
    private CANSparkMax driveMotor;
    private CANSparkMax angleMotor;

    // Encoders
    private CANCoder absEncoder;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder angleEncoder;

    // PID Controllers
    private SparkMaxPIDController anglePIDController;
    private SparkMaxPIDController drivePIDController;

    // State of the module
    private SwerveModuleState state;

    // Location of the module relative to robot center
    public Translation2d location;

    // Constants
    private final double angleMaxAccel = 1; //RPM/sec
    private final double angleMaxVel = 99999; //RPM
    private final double wheelRadius = 0.0508; //Meters

    public SwerveModule(int angleID, int driveID, int encoderID, double[] anglePID, double[] drivePID, double X, double Y, boolean invertDrive) {

        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(angleID, MotorType.kBrushless);

        absEncoder = new CANCoder(encoderID);
        driveEncoder = driveMotor.getEncoder();
        angleMotor.setInverted(false);
        angleEncoder = angleMotor.getEncoder();

        anglePIDController = angleMotor.getPIDController();
        drivePIDController = driveMotor.getPIDController();
        anglePIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        anglePIDController.setSmartMotionMaxAccel(angleMaxAccel, 0);
        anglePIDController.setSmartMotionMaxVelocity(angleMaxVel, 0);
        
        location = new Translation2d(X/2, Y/2);
    
        driveMotor.setInverted(invertDrive);
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setOpenLoopRampRate(0.5);

        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    }

    public void setState(SwerveModuleState setState) {
        state = SwerveModuleState.optimize(setState, getEncoder());

        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        anglePIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
    }

    public void configEncoder(double offset) {
        
        absEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
        absEncoder.configMagnetOffset(offset);

        angleEncoder.setPositionConversionFactor(360/12.8);
        angleEncoder.setPosition(getABSEncoder().getDegrees());

        driveEncoder.setVelocityConversionFactor(((2*Math.PI*wheelRadius)/60)/8.14);
        driveEncoder.setPositionConversionFactor(1/8.14);

    }

    public void configPID(double[] anglePID, double[] drivePID) {
        anglePIDController.setP(anglePID[0]);
        anglePIDController.setI(anglePID[1]);
        anglePIDController.setD(anglePID[2]);
        anglePIDController.setPositionPIDWrappingMinInput(0);
        anglePIDController.setPositionPIDWrappingMaxInput(360);
        anglePIDController.setPositionPIDWrappingEnabled(true);

        drivePIDController.setP(drivePID[0]);
        drivePIDController.setI(drivePID[1]);
        drivePIDController.setD(drivePID[2]);
    }

    public Rotation2d getABSEncoder() {
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    public Rotation2d getEncoder() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public SwerveModulePosition getModuleState() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getEncoder());
    }

    public void stopMotors() {
        angleMotor.stopMotor();
        driveMotor.stopMotor();
    }

}