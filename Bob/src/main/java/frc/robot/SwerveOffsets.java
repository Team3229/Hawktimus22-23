package frc.robot;

import java.io.FileWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveOffsets {

    private final String path = "/home/lvuser/";
    ModuleOffsets object = new ModuleOffsets();
    boolean resetOffsets = false;
    String frontLeftPath = path + "frontLeft.txt";
    String frontRightPath = path + "frontLeft.txt";
    String backLeftPath = path + "frontLeft.txt";
    String backRightPath = path + "frontLeft.txt";

    SwerveOffsets() {
        SmartDashboard.putBoolean("resetAngleOffsets", resetOffsets);
    }

    double[] calculateOffsets(double fL, double fR, double bL, double bR){
        // takes the current values, assumes they should be 0, and returns an [] of the new values to set them to after storing it for futute use
        // if old offsets < 90, its closer to 0. Else if > 90 & < 270, closer to 180. Else its > 270, so closer to 360
        double[] newOffsets = {
            fL < 90 ? -fL: fL > 90 & fL < 270 ? 180 - fL:360 - fL,
            fR < 90 ? -fR: fR > 90 & fR < 270 ? 180 - fR:360 - fR,
            bL < 90 ? -bL: bL > 90 & bL < 270 ? 180 - bL:360 - bL,
            bR < 90 ? -bR: bR > 90 & bR < 270 ? 180 - bR:360 - bR
        };
        writeOffsets(newOffsets);
        return newOffsets;

    }
    void writeOffsets(double[] newAngles) {
        
        object.frontLeft = newAngles[0];
        object.frontRight = newAngles[1];
        object.backLeft = newAngles[2];
        object.backRight = newAngles[3];

        // attempt to write the new offsets to the file, catches exceptions if failure
        try {
            FileWriter fLWriter = new FileWriter(frontLeftPath);
            fLWriter.write(String.valueOf(object.frontLeft));
            fLWriter.close();
            FileWriter fRWriter = new FileWriter(frontRightPath);
            fRWriter.write(String.valueOf(object.frontRight));
            fRWriter.close();
            FileWriter bLWriter = new FileWriter(backLeftPath);
            bLWriter.write(String.valueOf(object.backLeft));
            bLWriter.close();
            FileWriter bRWriter = new FileWriter(backRightPath);
            bRWriter.write(String.valueOf(object.backRight));
            bRWriter.close();
        } catch(IOException e){
            
        }
         
    }

    String readFile(File file) throws IOException {

        FileInputStream fis = null;
        int len = (int) file.length();
        byte[] bytes = new byte[len];

        try {

            fis = new FileInputStream(file);
            assert len == fis.read(bytes);

        } catch (IOException e) {
            fis.close();
            file.createNewFile();
            throw e;
        }

        fis.close();
        return new String(bytes, "UTF-8");

    }

    class ModuleOffsets {

        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;

    }
    
}