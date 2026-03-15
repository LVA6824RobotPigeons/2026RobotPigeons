package frc.robot.subsystems;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;

import java.util.ArrayList;
import java.util.TreeMap;

import static frc.robot.Constants.LEDs.kNumberOfLights;
import static frc.robot.Constants.LEDs.kStartLED;

public class LEDManager {

    private final TreeMap<Integer,RGBWColor> layers = new TreeMap<>();

    public void set(RGBWColor color, int zIndex) {
        layers.put(zIndex, color);
    }

    public void remove(RGBWColor color, int zIndex) {
        layers.remove(zIndex, color);
    }
    public RGBWColor getCurrentColor() {
        return layers.isEmpty() ? null : layers.lastEntry().getValue();
    }

    public Integer getCurrentZ() {
        return layers.isEmpty() ? null : layers.lastKey();
    }

    public boolean isTop(int zIndex) {
        return !layers.isEmpty() && layers.lastKey() == zIndex;
    }


} //ok