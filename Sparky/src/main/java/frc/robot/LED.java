//LEDEEZNUTZ

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED {
    
    private static Spark blinkin = new Spark(0);

    public static final double RAINBOW_rainbowPallete = -0.99;
    public static final double SOLID_red = 0.61;
    public static final double SOLID_redOrange = 0.63;
    public static final double SOLID_orange = 0.65;
    public static final double SOLID_gold = 0.67;
    public static final double SOLID_yellow = 0.69;
    public static final double SOLID_green = 0.77;
    public static final double SOLID_greenBlue = 0.79;
    public static final double SOLID_blue = 0.87;
    public static final double SOLID_purple = 0.91;
    public static final double SOLID_white = 0.93;
    public static final double SOLID_grey = 0.95;
    public static final double SOLID_darkGrey = 0.97;
    public static final double SOLID_black = 0.99;
    public static final double MULTICOLOR_sinelonPurpleGold = 0.55;
    public static final double MULTICOLOR_twinklePurpleGold = 0.51;
    public static final double FIXEDPATTERN_waveLava = -0.39;
    public static final double FIXEDPATTERN_waveOcean = -0.41;
    public static final double FIXEDPATTERN_strobeGold = -0.07;
    public static final double COLORONEPATTERN_strobePurple = 0.15;
    public static final double off = 0;

    LED(){}

    public static void setColor(double color) {
        if (color == 0) {
            blinkin.stopMotor();
            return;
        }
        blinkin.set(color);
    }

}