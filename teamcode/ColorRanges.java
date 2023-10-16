package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
public class ColorRanges {
    public enum ColorFromHue{
        RED,
        YELLOW,
        BLUE,
        INDETERMINATE
    }
    static final double SCALE_FACTOR = 255;
    public static int hue;
    public static ColorFromHue GetColor(int red, int green, int blue) {
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV((int) (red * SCALE_FACTOR), (int) (green * SCALE_FACTOR), (int) (blue * SCALE_FACTOR), hsvValues);
        hue = (int) hsvValues[0];
        /*
        if (hue >= 70 && hue <= 110) {
            return ColorFromHue.YELLOW;

         */
        if ((hue >= 330 && hue <= 360) || (hue >= 0 && hue <= 45))
            return ColorFromHue.RED;
        else if (hue >= 180 && hue <= 270) {
            return ColorFromHue.BLUE;
        }
        return ColorFromHue.INDETERMINATE;
    }
    public static int getHue(){
        return hue;
    }
}
