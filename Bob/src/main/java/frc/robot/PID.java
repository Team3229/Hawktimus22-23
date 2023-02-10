package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class PID {
    double[] pidValues = {0,0,0};
    String totalPath = "/home/lvuser/";
    PID(String path,double[] DefaultValues){
        totalPath += path;
        if(DefaultValues != null){
            pidValues = DefaultValues;
            writePID();
        } else {
            pidValues = readPID();
        }
    }
    void writePID(){
        try{
            FileWriter writer = new FileWriter(totalPath);
            writer.write(String.valueOf(pidValues[0])+"\n"+String.valueOf(pidValues[1])+"\n"+String.valueOf(pidValues[2]));
            writer.close();
        }catch(IOException e){
            System.out.println(e);
        }
    }
    double[] readPID(){
        double[] pid = {0,0,0};
        try{
            File file = new File(totalPath);
            Scanner scanner = new Scanner(file);
            pidValues[0] = Double.parseDouble(scanner.nextLine());
            pidValues[1] = Double.parseDouble(scanner.nextLine());
            pidValues[2] = Double.parseDouble(scanner.nextLine());
            scanner.close();
        } catch(IOException e){
            System.out.println(e);
        }
        return pid;
    }
}