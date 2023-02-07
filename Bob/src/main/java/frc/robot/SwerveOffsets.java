package frc.robot;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.Console;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.ObjectInputStream;
import java.io.Serializable;

import com.fasterxml.jackson.core.JsonParser;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveOffsets {

    private final String path = "/home/lvuser/";
    ModuleOffsets object = new ModuleOffsets();
    boolean resetOffsets = false;
    File frontLeft = new File(path + "frontLeft.txt");
    File frontRight = new File(path + "frontLeft.txt");
    File backLeft = new File(path + "frontLeft.txt");
    File backRight = new File(path + "frontLeft.txt");

    SwerveOffsets() {
        SmartDashboard.putBoolean("resetAngleOffsets", resetOffsets);
    }

    double[] calculateOffsets(String fL, String fR, String bL, String bR){
        // takes the current values, assumes they should be 0, and returns an [] of the new values to set them to after storing it for futute use
        ModuleOffsets oldOffsets = new ModuleOffsets(fL, fR, bL, bR);
        // if old offsets < 90, its closer to 0. Else if > 90 & < 270, closer to 180. Else its > 270, so closer to 360
        double[] newOffsets = {
            oldOffsets.frontLeft < 90 ? -oldOffsets.frontLeft: oldOffsets.frontLeft > 90 & oldOffsets.frontLeft < 270 ? 180 - oldOffsets.frontLeft:360 - oldOffsets.frontLeft,
            oldOffsets.frontRight < 90 ? -oldOffsets.frontRight: oldOffsets.frontRight > 90 & oldOffsets.frontRight < 270 ? 180 - oldOffsets.frontRight:360 - oldOffsets.frontRight,
            oldOffsets.backLeft < 90 ? -oldOffsets.backLeft: oldOffsets.backLeft > 90 & oldOffsets.backLeft < 270 ? 180 - oldOffsets.backLeft:360 - oldOffsets.backLeft,
            oldOffsets.backRight < 90 ? -oldOffsets.backRight: oldOffsets.backRight > 90 & oldOffsets.backRight < 270 ? 180 - oldOffsets.backRight:360 - oldOffsets.backRight
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

    void writeFile(String filePath, String toWrite) {

        FileOutputStream fos = null;
        try {
            fos = new FileOutputStream((path + "frontLeft.txt"));
            fos.write(toWrite.getBytes("UTF-8"));
            fos.close();
        } catch (IOException e) {
            System.out.println(e);

            
            
        }

    }

    class ModuleOffsets {

        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;

        ModuleOffsets(String fl, String fr, String bl, String br) {
            frontLeft = Double.parseDouble(fl);
            frontRight = Double.parseDouble(fr);
            backLeft = Double.parseDouble(bl);
            backRight = Double.parseDouble(br);
        }

    }
    
}