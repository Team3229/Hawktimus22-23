//Otters: 3229 Programming SubTeam

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
    // class for handling all dashboard things, to simplify everything.
    public Dashboard(){};
    public void putBool(String key, Boolean bool){
        SmartDashboard.putBoolean(key, bool);
    }
    public void putNumber(String key, int value){
        SmartDashboard.putNumber(key, value);
    }
    public void putNumber(String key, double value){
        SmartDashboard.putNumber(key, value);
    }
    public void putNumber(String key, float value){
        SmartDashboard.putNumber(key, value);
    }
    public void putNumber(String key, long value){
        SmartDashboard.putNumber(key, value);
    }
    public void putNumber(String key, short value){
        SmartDashboard.putNumber(key, value);
    }
    public void putNumberArray(String key, double[] value){
        SmartDashboard.putNumberArray(key, value);
    }
    public double readNumber(String key){
        return SmartDashboard.getNumber(key, 0);
    }
    public boolean readBool(String key){
        return SmartDashboard.getBoolean(key, false);
    }
}
