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

    void writeOffsets(double frontLeft, double frontRight, double backLeft, double backRight) {
        
        object.frontLeft = frontLeft;
        object.frontRight = frontRight;
        object.backLeft = backLeft;
        object.backRight = backRight;

        try {

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

            ObjectInputStream in = new ObjectInputStream(new FileInputStream(path));

            try {
                object = (ModuleOffsets) in.readObject();
            } catch (ClassNotFoundException e) {
                e.printStackTrace();
            }
            in.close();

            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
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