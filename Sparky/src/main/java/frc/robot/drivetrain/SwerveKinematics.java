package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

public class SwerveKinematics {
    
    // Instantiate each module
    private static SwerveModule frontLeftModule;
    private static SwerveModule frontRightModule;
    private static SwerveModule backLeftModule;
    private static SwerveModule backRightModule;
    public static ModuleOffsets offsets;

    // Create kinematics objects
    private static SwerveDriveKinematics kinematics;
    private static ChassisSpeeds chassisState;
    private static SwerveModuleState[] moduleStates;

    public SwerveModulePosition[] positions;
    public SwerveDrivePoseEstimator odometry;

    // Create gyro and rotation objects
    public AHRS navxGyro;
    public Rotation2d robotRotation = Rotation2d.fromDegrees(0);
    public boolean relativeMode = false;

    // PID
    private static final double[] anglePID = {0.01, 0.0001, 0};
    private static final double[] drivePID = {0.1, 0, 0};

    // Constants
    private static final double robotWidth = 0.762;
    private static final double maxModuleSpeed = 12; //meters/sec
    private static final double maxChassisRotationSpeed = 0.75; //radians/sec

    public SwerveKinematics() {

        frontLeftModule = new SwerveModule(1, 2, 9, anglePID, drivePID, robotWidth, robotWidth, false);
        frontRightModule = new SwerveModule(6, 5, 10, anglePID, drivePID, robotWidth, -robotWidth, false);
        backLeftModule = new SwerveModule(4, 3, 11, anglePID, drivePID, -robotWidth, robotWidth, false);
        backRightModule = new SwerveModule(8, 7, 12, anglePID, drivePID, -robotWidth, -robotWidth, true);

        offsets = new ModuleOffsets();
        configEncoders(offsets.read());

        navxGyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(frontLeftModule.location, frontRightModule.location, backLeftModule.location, backRightModule.location);

        positions = new SwerveModulePosition[4];

        for (int i = 0; i < 4; i++) {
            positions[i] = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
        }

        odometry = new SwerveDrivePoseEstimator(kinematics, robotRotation, positions, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    }

    public void drive(double X, double Y, double Z) {
        if (relativeMode) {
            robotRotation = Rotation2d.fromDegrees(180);
        } else {
            robotRotation = Rotation2d.fromDegrees(navxGyro.getYaw()*-1);
        }

        chassisState = ChassisSpeeds.fromFieldRelativeSpeeds(Y*12, X*12, Z*maxChassisRotationSpeed*12, robotRotation);
        moduleStates = kinematics.toSwerveModuleStates(chassisState);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxModuleSpeed);

        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);

    }

    public void drive(ChassisSpeeds speeds) {

        robotRotation = Rotation2d.fromDegrees(navxGyro.getYaw()*-1);
        moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxModuleSpeed);

        frontLeftModule.setState(moduleStates[0]);
        frontRightModule.setState(moduleStates[1]);
        backLeftModule.setState(moduleStates[2]);
        backRightModule.setState(moduleStates[3]);
    }

    public void configEncoders() {
        double[] offset = offsets.read();
        frontLeftModule.configEncoder(offset[0]);
        frontRightModule.configEncoder(offset[1]);
        backLeftModule.configEncoder(offset[2]);
        backRightModule.configEncoder(offset[3]);
    }

    private void configEncoders(double[] offset) {
        frontLeftModule.configEncoder(offset[0]);
        frontRightModule.configEncoder(offset[1]);
        backLeftModule.configEncoder(offset[2]);
        backRightModule.configEncoder(offset[3]);
    }

    public Rotation2d[] absEncoderValues() {
        return new Rotation2d[] {frontLeftModule.getABSEncoder(), frontRightModule.getABSEncoder(), backLeftModule.getABSEncoder(), backRightModule.getABSEncoder()};
    }

    public void configPIDS() {
        frontLeftModule.configPID(anglePID, drivePID);
        frontRightModule.configPID(anglePID, drivePID);
        backLeftModule.configPID(anglePID, drivePID);
        backRightModule.configPID(anglePID, drivePID);
    }

    public void fixOffsets() {
        configEncoders(offsets.calculateOffsets(frontLeftModule.getABSEncoder(), frontRightModule.getABSEncoder(), backLeftModule.getABSEncoder(), backRightModule.getABSEncoder()));
    }

    public void zeroGyro() {
        navxGyro.zeroYaw();
        navxGyro.calibrate();
    }

    public double getPitch() {
        return navxGyro.getPitch();
    }

    public void updateOdometry() {
        positions[0] = frontLeftModule.getModuleState();
        positions[1] = frontRightModule.getModuleState();
        positions[2] = backLeftModule.getModuleState();
        positions[3] = backRightModule.getModuleState();
        odometry.update(robotRotation, positions);
    }

    public void correctOdometry(Pose2d position, double timeStamp) {
        odometry.addVisionMeasurement(position, timeStamp);
    }

    public void stop() {
        frontLeftModule.stopMotors();
        frontRightModule.stopMotors();
        backLeftModule.stopMotors();
        backRightModule.stopMotors();
    }

}