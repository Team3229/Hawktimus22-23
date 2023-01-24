// Otter: Nathan Manhardt (3229 Software Captain)
// Co-Otters: 3229 Software Team

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;

public class SwerveKinematics {
    //126.9140625     255.5859375    346.728515625    6.416015625
    double[] offsets = {53.0859375,-255.5859375,-346.728515625,-6.416015625};
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

    double[] anglePID = {0.3, 0, 0.00001};
    double[] drivePID = {0, 0, 0};
    final double anglePosTolerance = 1;
    final double angleVelTolerance = 1;

    final double L = 0.58705;
    final double W = 0.58705;
    final double maxSpeedMetersPerSecond = 7;
    double maxRadiansPerSecond = 0.75;

    double[] encoderValues = {0, 0, 0, 0};

    boolean rotating = false;

    public SwerveKinematics() {

        frontLeftModule = new SwerveModule(1, 2, 9, anglePID, drivePID, L, W, false);
        frontRightModule = new SwerveModule(5, 6, 10, anglePID, drivePID,L, -W, true);
        backLeftModule = new SwerveModule(3, 4, 11, anglePID, drivePID, -L, W, true);
        backRightModule = new SwerveModule(7, 8, 12, anglePID, drivePID, -L, -W, false);

        frontLeftModule.ConfigEncoder(offsets[0]);
        frontRightModule.ConfigEncoder(offsets[1]);
        backLeftModule.ConfigEncoder(offsets[2]);
        backRightModule.ConfigEncoder(offsets[3]);

        navxGyro = new AHRS(SPI.Port.kMXP);

        kinematicsObject = new SwerveDriveKinematics(frontLeftModule.location, frontRightModule.location, backLeftModule.location, backRightModule.location);

        utils = new Utils();

    }

    void Drive(double X, double Y, double Z) {

        rotating = ((Math.abs(Z) > 0) ? true : false);

        robotRotation = utils.convertAngle(navxGyro.getYaw()*-1);
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(Y, X, Z*maxRadiansPerSecond, Rotation2d.fromDegrees(robotRotation));
        states = kinematicsObject.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMetersPerSecond);

        frontLeftModule.SetState(states[0]);
        frontRightModule.SetState(states[1]);
        backLeftModule.SetState(states[2]);
        backRightModule.SetState(states[3]);


    }

    double[] EncoderValues() {

        encoderValues[0] = frontLeftModule.GetAbsoluteEncoder();
        encoderValues[1] = frontRightModule.GetAbsoluteEncoder();
        encoderValues[2] = backLeftModule.GetAbsoluteEncoder();
        encoderValues[3] = backRightModule.GetAbsoluteEncoder();
        robotRotation = utils.convertAngle(navxGyro.getYaw()*-1);
        return encoderValues;

    }

    void ConfigPIDS() {

        frontLeftModule.ConfigPID(anglePID, drivePID);
        frontRightModule.ConfigPID(anglePID, drivePID);
        backLeftModule.ConfigPID(anglePID, drivePID);
        backRightModule.ConfigPID(anglePID, drivePID);
    }

    void Stop() {

        frontLeftModule.Stop();
        frontRightModule.Stop();
        backLeftModule.Stop();
        backRightModule.Stop();

    }

}