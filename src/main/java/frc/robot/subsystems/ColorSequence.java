package frc.robot.subsystems;

import com.ctre.phoenix6.signals.RGBWColor;

import java.util.ArrayList;
import java.util.function.BiConsumer;

public class ColorSequence {

    public static ArrayList<ColorSequence> colorSequences = new ArrayList<>();
    public static void process(int refreshRate) {
        for(ColorSequence seq : colorSequences) {
            seq.tick(refreshRate);
        }
    }

    private final RGBWColor[] colors;
    private final BiConsumer<RGBWColor, Integer> colorSink;
    private final long stepIntervalMs;
    private final int zIndex;

    public ColorSequence(LED8 led, RGBWColor[] colors, long speed, int zIndex) {
        this((color, layer) -> led.setColor(color, layer), colors, speed, zIndex);
    }

    ColorSequence(BiConsumer<RGBWColor, Integer> colorSink, RGBWColor[] colors, long speed, int zIndex) {
        this.colorSink = colorSink;
        this.colors = colors;
        this.stepIntervalMs = Math.max(1, speed);
        this.zIndex = zIndex;
        colorSequences.add(this);
    }

    private long elapsedMs = 0;
    public void tick(int refreshRateMs) {
        if (colors.length == 0) return;
        int colorIndex = (int)((elapsedMs / stepIntervalMs) % colors.length);
        colorSink.accept(colors[colorIndex], zIndex);
        elapsedMs += Math.max(1, refreshRateMs);
    }

    public void stop() {colorSequences.remove(this);}

}
