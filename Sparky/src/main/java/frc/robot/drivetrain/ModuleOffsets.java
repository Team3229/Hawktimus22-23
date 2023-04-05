//Otters: 3229 Programming SubTeam

package frc.robot.drivetrain;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ModuleOffsets {

    private final String path = "/home/lvuser/";
    private final String[] fileNames = {"frontLeft.txt", "frontRight.txt", "backLeft.txt", "backRight.txt"};

    public ModuleOffsets() {
        SmartDashboard.putBoolean("resetAngleOffsets", false);
    }

    public double[] calculateOffsets(Rotation2d fL, Rotation2d fR, Rotation2d bL, Rotation2d bR) {
        // takes the current values, assumes they should be 0, and returns an [] of the new values to set them to after storing it for futute use
        // if old offsets < 90, its closer to 0. Else if > 90 & < 270, closer to 180. Else its > 270, so closer to 360
        // if the values passed, which are the current values, and should be zero.
        double[] currentValues = read();
        double[] newOffsets = {
            currentValues[0] + (fL.getDegrees() < 90 ? -fL.getDegrees(): fL.getDegrees() > 90 && fL.getDegrees() < 270 ? 180 - fL.getDegrees():360 - fL.getDegrees()),
            currentValues[1] + (fR.getDegrees() < 90 ? -fR.getDegrees(): fR.getDegrees() > 90 && fR.getDegrees() < 270 ? 180 - fR.getDegrees():360 - fR.getDegrees()),
            currentValues[2] + (bL.getDegrees() < 90 ? -bL.getDegrees(): bL.getDegrees() > 90 && bL.getDegrees() < 270 ? 180 - bL.getDegrees():360 - bL.getDegrees()),
            currentValues[3] + (bR.getDegrees() < 90 ? -bR.getDegrees(): bR.getDegrees() > 90 && bR.getDegrees() < 270 ? 180 - bR.getDegrees():360 - bR.getDegrees())
        };
        writeOffsets(newOffsets);
        return newOffsets;
    }

    private void writeOffsets(double[] newAngles) {
        // attempt to write the new offsets to the file, catches exceptions if failure
        // writes strings to the file, need to parse it to doubl.getDegrees()e once read.
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

    public double[] read() {
        // returns the currently stored values, 0,0,0,0 if none, as doubl.getDegrees()es.
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
