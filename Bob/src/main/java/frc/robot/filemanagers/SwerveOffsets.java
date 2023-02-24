//Otters: 3229 Programming SubTeam

package frc.robot.filemanagers;

import java.io.*;
import java.util.Scanner;

import frc.robot.Dashboard;


public class SwerveOffsets {

    private final String path = "/home/lvuser/";
    private final String[] fileNames = {"frontLeft.txt", "frontRight.txt", "backLeft.txt", "backRight.txt"};

    Dashboard dash = new Dashboard();

    public SwerveOffsets() {
        dash.putBool("resetAngleOffsets", false);
    }

    public double[] calculateOffsets(double fL, double fR, double bL, double bR) {
        // takes the current values, assumes they should be 0, and returns an [] of the new values to set them to after storing it for futute use
        // if old offsets < 90, its closer to 0. Else if > 90 & < 270, closer to 180. Else its > 270, so closer to 360
        // if the values passed, which are the current values, and should be zero.
        double[] currentValues = readFiles();
        double[] newOffsets = {
            currentValues[0] + (fL < 90 ? -fL: fL > 90 && fL < 270 ? 180 - fL:360 - fL),
            currentValues[1] + (fR < 90 ? -fR: fR > 90 && fR < 270 ? 180 - fR:360 - fR),
            currentValues[2] + (bL < 90 ? -bL: bL > 90 && bL < 270 ? 180 - bL:360 - bL),
            currentValues[3] + (bR < 90 ? -bR: bR > 90 && bR < 270 ? 180 - bR:360 - bR)
        };
        writeOffsets(newOffsets);
        return newOffsets;
    }

    private void writeOffsets(double[] newAngles) {
        // attempt to write the new offsets to the file, catches exceptions if failure
        // writes strings to the file, need to parse it to double once read.
        try {
            for (int i = 0; i < fileNames.length; i++) {
                FileWriter writer = new FileWriter(path + fileNames[i]);
                writer.write(String.valueOf(newAngles[i]));
                writer.close();
            }
        } catch(IOException e){
            System.out.println("An error occurred while writing the swerve angle offsets to file.");
            e.printStackTrace();
        }
    }

    public double[] readFiles() {
        // returns the currently stored values, 0,0,0,0 if none, as doubles.
        double[] values = {0,0,0,0};
        try {
            for (int i = 0; i < fileNames.length; i++) {
                File file = new File(path + fileNames[i]);
                Scanner scanner = new Scanner(file);
                if (scanner.hasNextLine()) {
                    values[i] = Double.parseDouble(scanner.nextLine());
                }
                scanner.close();
            }
        } catch(IOException e){
            System.out.println("An error occurred while reading the swerve angle offsets from file.");
            e.printStackTrace();
        }
        return values;
    }
}
