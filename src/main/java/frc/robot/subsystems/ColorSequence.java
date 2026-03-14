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

    private RGBWColor[] colors;
    private LED8 led;
    private long speed = 100; // (As a percent) Multiplier for amount of times to tick every RobotPeriodic call
    private int zIndex;

    public ColorSequence(LED8 led, RGBWColor[] colors, long speed, int zIndex) {
        this.speed = speed;
        this.led = led;
        this.colors = colors;
        this.zIndex = zIndex;
        colorSequences.add(this);
    }

    private long counter = 0;
    public void tick(int refreshRate) {
        if (colors.length == 0) return;
        long phase = counter * refreshRate % Math.max(1, speed);
        int colorIndex = (int)(phase % colors.length);
        led.setColor(colors[colorIndex], zIndex);
        counter++;
    }

    public void stop() {colorSequences.remove(this);}

}