//LEDEEZNUTZ

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

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
    public static final float MULTICOLOR_sinelonPurpleGold = 0.55f;
    public static final float MULTICOLOR_twinklePurpleGold = 0.51f;
    public static final float MULTICOLOR_sparklePurpleGold = 0.37f;
    public static final float MULTICOLOR_colorWavesPurpleGold = 0.53f;
    public static final float MULTICOLOR_bpmPurpleGold = 0.43f;
    public static final float FIXEDPATTERN_waveLava = -0.39f;
    public static final float FIXEDPATTERN_waveOcean = -0.41f;
    public static final float FIXEDPATTERN_strobeGold = -0.07f;
    public static final float COLORONEPATTERN_strobePurple = 0.15f;
    public static final float off = 0f;

    public float currentColor = 0f;

    LED(){}

    public void setColor() {
        if (currentColor == 0) {
            blinkin.stopMotor();
            return;
        }
        blinkin.set(currentColor);
    }

}