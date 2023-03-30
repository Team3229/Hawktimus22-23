//Otters: 3229 Programming Sub-Team

package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;
import frc.robot.filemanagers.SwerveOffsets;

public class SwerveKinematics {

    SwerveModule frontLeftModule;
    SwerveModule frontRightModule;
    SwerveModule backLeftModule;
    SwerveModule backRightModule;
    public AHRS navxGyro;
    SwerveDriveKinematics kinematicsObject;
    ChassisSpeeds speeds;
    SwerveModuleState[] states;
    public double robotRotation = 0;
    SwerveOffsets offset;
    // double[] anglePID = {0.003, 0.0002, 0.00001};
    // kP = 0.06

    public double[] anglePID = {0.01, 0.0002, 0};
    public double[] drivePIDFF = {0, 0, 0, 0};

    final double L = 0.594;
    final double W = 0.594;
    final double maxSpeedMetersPerSecond = 1;
    double maxRadiansPerSecond = 0.75;

    public boolean relativeMode = false;

    double[] encoderValues = {0, 0, 0, 0};

    double[] initialOffsets;

    public SwerveKinematics() {

        offset = new SwerveOffsets();

        frontLeftModule = new SwerveModule(2, 1, 9, anglePID, drivePIDFF, L, W, false);
        frontRightModule = new SwerveModule(5, 6, 10, anglePID, drivePIDFF, L, -W, false);
        backLeftModule = new SwerveModule(3, 4, 11, anglePID, drivePIDFF, -L, W, false);
        backRightModule = new SwerveModule(7, 8, 12, anglePID, drivePIDFF, -L, -W, true);

        initialOffsets = offset.readFiles();
        frontLeftModule.configEncoder(initialOffsets[0]);
        frontRightModule.configEncoder(initialOffsets[1]);
        backLeftModule.configEncoder(initialOffsets[2]);
        backRightModule.configEncoder(initialOffsets[3]);

        navxGyro = new AHRS(SPI.Port.kMXP);

        kinematicsObject = new SwerveDriveKinematics(frontLeftModule.location, frontRightModule.location, backLeftModule.location, backRightModule.location);

    }

    public void drive(double X, double Y, double Z) {

        if (relativeMode) {
            robotRotation = 180;
        } else {
            robotRotation = MathUtil.inputModulus(navxGyro.getYaw()*-1, 0, 360);
        }
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Y, X, Z*maxRadiansPerSecond, Rotation2d.fromDegrees(robotRotation));
        states = kinematicsObject.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);

        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);

    }

    public double[] encoderValues() {

        encoderValues[0] = frontLeftModule.getCANCoder();
        encoderValues[1] = frontRightModule.getCANCoder();
        encoderValues[2] = backLeftModule.getCANCoder();
        encoderValues[3] = backRightModule.getCANCoder();
        robotRotation = MathUtil.inputModulus(navxGyro.getYaw()*-1, 0, 360);
        return encoderValues;

    }

    public void configPIDS() {

        frontLeftModule.configPID(anglePID, drivePIDFF);
        frontRightModule.configPID(anglePID, drivePIDFF);
        backLeftModule.configPID(anglePID, drivePIDFF);
        backRightModule.configPID(anglePID, drivePIDFF);
    }

    public void fixOffsets() {

        double[] offsets = offset.calculateOffsets(frontLeftModule.getCANCoder(), frontRightModule.getCANCoder(), backLeftModule.getCANCoder(), backRightModule.getCANCoder());
        frontLeftModule.configEncoder(offsets[0]);
        frontRightModule.configEncoder(offsets[1]);
        backLeftModule.configEncoder(offsets[2]);
        backRightModule.configEncoder(offsets[3]);

    }

    public void configEncoders() {
        double[] offsets = offset.readFiles();
        frontLeftModule.configEncoder(offsets[0]);
        frontRightModule.configEncoder(offsets[1]);
        backLeftModule.configEncoder(offsets[2]);
        backRightModule.configEncoder(offsets[3]);
    }

    public void stop() {

        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();

    }

}