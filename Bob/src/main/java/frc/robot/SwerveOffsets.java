package frc.robot;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.io.ObjectInputStream;
import java.io.Serializable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveOffsets {

    private final String path = "/home/lvuser/offsets.txt";
    ModuleOffsets object = new ModuleOffsets();
    boolean resetOffsets = false;

    SwerveOffsets() {
        SmartDashboard.putBoolean("resetAngleOffsets", resetOffsets);
    }

    double[] calculateOffsets(double fL, double fR, double bL, double bR){
        // takes the current values, assumes they should be 0, and returns an [] of the new values to set them to after storing it for futute use
        ModuleOffsets oldOffsets = readOffsets();
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

        try {
        // attempt to write the new offsets to the file, catches exceptions if failure
        ObjectOutputStream out = new ObjectOutputStream(new FileOutputStream(path));

        out.writeObject(object);
        out.close();

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    ModuleOffsets readOffsets() {

        try {
            // attempt to read the offsets previously stored to be used later.
            ObjectInputStream in = new ObjectInputStream(new FileInputStream(path));

            object = (ModuleOffsets) in.readObject();
            in.close();

            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }catch (ClassNotFoundException e) {
                e.printStackTrace();
            }

        return object;

    }

    class ModuleOffsets implements Serializable {

        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;

    }
    
}