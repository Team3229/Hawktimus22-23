//Otters: 3229 Programming Sub-Team

package frc.robot.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.filemanagers.PID;
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

    //double[] anglePID = {0.003, 0.0002, 0.00001};
    //double[] drivePID = {0, 0, 0};
    public PID anglePID = new PID("anglePID.txt", new double[] {0.003,0.0002,0.00001});
    public PID drivePID = new PID("drivePID.txt", new double[] {0,0,0});

    final double L = 0.594;
    final double W = 0.594;
    final double maxSpeedMetersPerSecond = 7;
    double maxRadiansPerSecond = 0.75;

    double[] encoderValues = {0, 0, 0, 0};

    boolean rotating = false;
    double[] initialOffsets;

    public SwerveKinematics() {

        offset = new SwerveOffsets();

        frontLeftModule = new SwerveModule(2, 1, 9, anglePID.pidValues, drivePID.pidValues, L, W, false);
        frontRightModule = new SwerveModule(5, 6, 10, anglePID.pidValues, drivePID.pidValues,L, -W, false);
        backLeftModule = new SwerveModule(3, 4, 11, anglePID.pidValues, drivePID.pidValues, -L, W, false);
        backRightModule = new SwerveModule(7, 8, 12, anglePID.pidValues, drivePID.pidValues, -L, -W, true);

        initialOffsets = offset.readFiles();
        frontLeftModule.configEncoder(initialOffsets[0]);
        frontRightModule.configEncoder(initialOffsets[1]);
        backLeftModule.configEncoder(initialOffsets[2]);
        backRightModule.configEncoder(initialOffsets[3]);

        navxGyro = new AHRS(SPI.Port.kMXP);

        kinematicsObject = new SwerveDriveKinematics(frontLeftModule.location, frontRightModule.location, backLeftModule.location, backRightModule.location);

    }

    public void drive(double X, double Y, double Z) {

        rotating = Math.abs(Z) > 0;

        robotRotation = MathUtil.inputModulus(navxGyro.getYaw()*-1, 0, 360);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Y, X, Z*maxRadiansPerSecond, Rotation2d.fromDegrees(robotRotation));
        states = kinematicsObject.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);

        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);


    }

    public double[] encoderValues() {

        encoderValues[0] = frontLeftModule.getEncoder();
        encoderValues[1] = frontRightModule.getEncoder();
        encoderValues[2] = backLeftModule.getEncoder();
        encoderValues[3] = backRightModule.getEncoder();
        robotRotation = MathUtil.inputModulus(navxGyro.getYaw()*-1, 0, 360);
        return encoderValues;

    }

    public void configPIDS() {

        frontLeftModule.configPID(anglePID.pidValues, drivePID.pidValues);
        frontRightModule.configPID(anglePID.pidValues, drivePID.pidValues);
        backLeftModule.configPID(anglePID.pidValues, drivePID.pidValues);
        backRightModule.configPID(anglePID.pidValues, drivePID.pidValues);
    }

    public void fixOffsets() {

        double[] offsets = offset.calculateOffsets(frontLeftModule.getEncoder(), frontRightModule.getEncoder(), backLeftModule.getEncoder(), backRightModule.getEncoder());
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