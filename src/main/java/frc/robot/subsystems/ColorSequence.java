package frc.robot.subsystems;

import com.ctre.phoenix6.signals.RGBWColor;

import java.util.ArrayList;

public class ColorSequence {

    public static ArrayList<ColorSequence> colorSequences = new ArrayList<>();
    public static void process(int refreshRate) {
        for(ColorSequence seq : colorSequences) {
            seq.tick(refreshRate);
        }
    }

    private final RGBWColor[] colors;
    private final LED8 led;
    private final long stepIntervalMs;
    private final int zIndex;

    public ColorSequence(LED8 led, RGBWColor[] colors, long speed, int zIndex) {
        this.led = led;
        this.colors = colors;
        this.stepIntervalMs = Math.max(1, speed);
        this.zIndex = zIndex;
        colorSequences.add(this);
    }

    private long elapsedMs = 0;
    public void tick(int refreshRateMs) {
        /*
        * fixed ticking to be time based
        * old math was funky, and w/speed=400 and colors.length=2, phase is always even (0,20,40...), so phase % 2 == 0 forever. not good
        * now we tie to time. very nice. :)
        */
        if (colors.length == 0) return;
        int colorIndex = (int)((elapsedMs / stepIntervalMs) % colors.length);
        led.setColor(colors[colorIndex], zIndex);
        elapsedMs += Math.max(1, refreshRateMs);
    }

    public void stop() {colorSequences.remove(this);}

}
