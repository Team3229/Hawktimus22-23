package frc.robot;

import java.io.FileWriter;
import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveOffsets {

    private final String path = "/home/lvuser/";
    boolean resetOffsets = false;
    String frontLeftPath = path + "frontLeft.txt";
    String frontRightPath = path + "frontRight.txt";
    String backLeftPath = path + "backLeft.txt";
    String backRightPath = path + "backRight.txt";

    SwerveOffsets() {
        SmartDashboard.putBoolean("resetAngleOffsets", resetOffsets);
    }

    double[] calculateOffsets(double fL, double fR, double bL, double bR) {
        // takes the current values, assumes they should be 0, and returns an [] of the new values to set them to after storing it for futute use
        // if old offsets < 90, its closer to 0. Else if > 90 & < 270, closer to 180. Else its > 270, so closer to 360
        // if the values passed, which are the current values, and should be zero.
        double[] currentValues = readFiles();
        double[] newOffsets = {
            currentValues[0] + fL < 90 ? -fL: fL > 90 & fL < 270 ? 180 - fL:360 - fL,
            currentValues[1] + fR < 90 ? -fR: fR > 90 & fR < 270 ? 180 - fR:360 - fR,
            currentValues[2] + bL < 90 ? -bL: bL > 90 & bL < 270 ? 180 - bL:360 - bL,
            currentValues[3] + bR < 90 ? -bR: bR > 90 & bR < 270 ? 180 - bR:360 - bR
        };
        writeOffsets(newOffsets);
        return newOffsets;

    }
    void writeOffsets(double[] newAngles) {


        // attempt to write the new offsets to the file, catches exceptions if failure
        // writes strings to the file, need to parse it to double once read.
        try {
            FileWriter fLWriter = new FileWriter(frontLeftPath);
            fLWriter.write(String.valueOf(newAngles[0]));
            fLWriter.close();
            FileWriter fRWriter = new FileWriter(frontRightPath);
            fRWriter.write(String.valueOf(newAngles[1]));
            fRWriter.close();
            FileWriter bLWriter = new FileWriter(backLeftPath);
            bLWriter.write(String.valueOf(newAngles[2]));
            bLWriter.close();
            FileWriter bRWriter = new FileWriter(backRightPath);
            bRWriter.write(String.valueOf(newAngles[3]));
            bRWriter.close();
        } catch(IOException e){
            System.out.println(e);
        }
         
    }

    double[] readFiles() {
        // returns the currently stored values, 0,0,0,0 if none, as doubles.
        double[] values = {0,0,0,0};
        try {
            File fLF = new File(frontLeftPath);
            File fRF = new File(frontRightPath);
            File bLF = new File(backLeftPath);
            File bRF = new File(backRightPath);
            Scanner fLFR = new Scanner(fLF);
            Scanner fRFR = new Scanner(fRF);
            Scanner bLFR = new Scanner(bLF);
            Scanner bRFR = new Scanner(bRF);
            values[0] = Double.parseDouble(fLFR.nextLine());
            values[1] = Double.parseDouble(fRFR.nextLine());
            values[2] = Double.parseDouble(bLFR.nextLine());
            values[3] = Double.parseDouble(bRFR.nextLine());
            fLFR.close();
            fRFR.close();
            bLFR.close();
            bRFR.close();
        } catch(IOException e){

        }
        return values;
    }
}