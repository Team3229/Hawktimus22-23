//Otters: 3229 Programming SubTeam

package frc.robot.filemanagers;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class PID {

    public double[] pidValues = {0.0, 0.0, 0.0};
    private String totalPath = "/home/lvuser/";

    public PID(String path, double[] defaultValues) {
        totalPath += path;
        if (defaultValues != null) {
            pidValues = defaultValues;
            writePID();
        } else {
            pidValues = readPID();
        }
    }

    public void writePID() {
        try {
            FileWriter writer = new FileWriter(totalPath);
            writer.write(pidValues[0] + "\n" + pidValues[1] + "\n" + pidValues[2]);
            writer.close();
        } catch (IOException e) {
            System.out.println(e);
        }
    }

    private double[] readPID() {
        double[] pid = {0.0, 0.0, 0.0};
        try {
            File file = new File(totalPath);
            Scanner scanner = new Scanner(file);
            pid[0] = Double.parseDouble(scanner.nextLine());
            pid[1] = Double.parseDouble(scanner.nextLine());
            pid[2] = Double.parseDouble(scanner.nextLine());
            scanner.close();
        } catch (IOException e) {
            System.out.println(e);
        }
        return pid;
    }

    // in order of P I D
    // up value till it oscillates equally, never stopping or speeding up. Then divide it by two
    // Repeat for all three in the correct order as said above
    // when bool on dashboard is on, each tick (20ms) call this. Track its furthest point from the setpoint

    public double[] getPIDValues() {
        return pidValues;
    }

    public void setPIDValues(double[] newValues) {
        pidValues = newValues;
        writePID();
    }

    public void trackFurthestPointFromSetpoint() {
        // TODO: Implement tracking of furthest point from setpoint
    }

}
