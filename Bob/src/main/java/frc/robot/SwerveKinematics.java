// Otter: Nathan Manhardt (3229 Software Captain)
// Co-Otters: 3229 Software Team

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;

public class SwerveKinematics {

    SwerveModule frontLeftModule;
    SwerveModule frontRightModule;
    SwerveModule backLeftModule;
    SwerveModule backRightModule;
    AHRS navxGyro;
    SwerveDriveKinematics kinematicsObject;
    ChassisSpeeds speeds;
    SwerveModuleState[] states;
    double robotRotation = 0;
    Utils utils;
    SwerveOffsets offset;

    //double[] anglePID = {0.003, 0.0002, 0.00001};
    //double[] drivePID = {0, 0, 0};
    PID anglePID = new PID("anglePID.txt", new double[] {0.003,0.0002,0.00001});
    PID drivePID = new PID("drivePID.txt",new double[] {0,0,0});

    final double L = 0.594;
    final double W = 0.594;
    final double maxSpeedMetersPerSecond = 7;
    double maxRadiansPerSecond = 0.75;

    double[] encoderValues = {0, 0, 0, 0};

    boolean rotating = false;
    double[] initialOffsets;

    public SwerveKinematics() {

        utils = new Utils();
        offset = new SwerveOffsets();

        frontLeftModule = new SwerveModule(2, 1, 9, anglePID.pidValues, drivePID.pidValues, L, W, false);
        frontRightModule = new SwerveModule(5, 6, 10, anglePID.pidValues, drivePID.pidValues,L, -W, false);
        backLeftModule = new SwerveModule(3, 4, 11, anglePID.pidValues, drivePID.pidValues, -L, W, false);
        backRightModule = new SwerveModule(7, 8, 12, anglePID.pidValues, drivePID.pidValues, -L, -W, true);

        initialOffsets = offset.readFiles();
        frontLeftModule.ConfigEncoder(initialOffsets[0]);
        frontRightModule.ConfigEncoder(initialOffsets[1]);
        backLeftModule.ConfigEncoder(initialOffsets[2]);
        backRightModule.ConfigEncoder(initialOffsets[3]);

        navxGyro = new AHRS(SPI.Port.kMXP);

        kinematicsObject = new SwerveDriveKinematics(frontLeftModule.location, frontRightModule.location, backLeftModule.location, backRightModule.location);

    }

    void Drive(double X, double Y, double Z) {

        rotating = ((Math.abs(Z) > 0) ? true : false);

        robotRotation = MathUtil.inputModulus(navxGyro.getYaw()*-1, 0, 360);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Y, X, Z*maxRadiansPerSecond, Rotation2d.fromDegrees(robotRotation));
        states = kinematicsObject.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);

        frontLeftModule.SetState(states[0]);
        frontRightModule.SetState(states[1]);
        backLeftModule.SetState(states[2]);
        backRightModule.SetState(states[3]);


    }

    double[] EncoderValues() {

        encoderValues[0] = frontLeftModule.GetEncoder();
        encoderValues[1] = frontRightModule.GetEncoder();
        encoderValues[2] = backLeftModule.GetEncoder();
        encoderValues[3] = backRightModule.GetEncoder();
        robotRotation = MathUtil.inputModulus(navxGyro.getYaw()*-1, 0, 360);
        return encoderValues;

    }

    void ConfigPIDS() {

        frontLeftModule.ConfigPID(anglePID.pidValues, drivePID.pidValues);
        frontRightModule.ConfigPID(anglePID.pidValues, drivePID.pidValues);
        backLeftModule.ConfigPID(anglePID.pidValues, drivePID.pidValues);
        backRightModule.ConfigPID(anglePID.pidValues, drivePID.pidValues);
    }

    void fixOffsets() {

        double[] offsets = offset.calculateOffsets(frontLeftModule.GetEncoder(), frontRightModule.GetEncoder(), backLeftModule.GetEncoder(), backRightModule.GetEncoder());
        frontLeftModule.ConfigEncoder(offsets[0]);
        frontRightModule.ConfigEncoder(offsets[1]);
        backLeftModule.ConfigEncoder(offsets[2]);
        backRightModule.ConfigEncoder(offsets[3]);

    }

    void Stop() {

        frontLeftModule.Stop();
        frontRightModule.Stop();
        backLeftModule.Stop();
        backRightModule.Stop();

    }

}