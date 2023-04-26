//LEDEEZNUTZ

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

/** LED class for controlling LED's on the robot */
public class LED {
    
    private static Spark blinkin = new Spark(0);

    public static final float RAINBOW_rainbowPallete = -0.99f;
    public static final float SOLID_red = 0.61f;
    public static final float SOLID_redOrange = 0.63f;
    public static final float SOLID_orange = 0.65f;
    public static final float SOLID_gold = 0.67f;
    public static final float SOLID_yellow = 0.69f;
    public static final float SOLID_green = 0.77f;
    public static final float SOLID_greenBlue = 0.79f;
    public static final float SOLID_blue = 0.87f;
    public static final float SOLID_purple = 0.91f;
    public static final float SOLID_white = 0.93f;
    public static final float SOLID_grey = 0.95f;
    public static final float SOLID_darkGrey = 0.97f;
    public static final float SOLID_black = 0.99f;
    public static final float MULTICOLOR_sinelon = 0.55f;
    public static final float MULTICOLOR_twinkle = 0.51f;
    public static final float MULTICOLOR_sparkle = 0.37f;
    public static final float MULTICOLOR_colorWaves = 0.53f;
    public static final float MULTICOLOR_bpm = 0.43f;
    public static final float FIXEDPATTERN_waveLava = -0.39f;
    public static final float FIXEDPATTERN_waveOcean = -0.41f;
    public static final float STROBE_gold = -0.07f;
    public static final float STROBE_purple = 0.15f;
    public static final float off = 0f;

    public static float currentColor = 0f;

    /** Sets the color of the LED's */
    public static void setColor() {
        if (currentColor == 0f) {
            blinkin.stopMotor();
            return;
        }
        blinkin.set(currentColor);
    }

}