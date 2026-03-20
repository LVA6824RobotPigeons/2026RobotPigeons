package frc.robot.subsystems.led8;

import frc.robot.subsystems.led8.Colors.ComplexColor;

import java.util.TreeMap;


public class LEDManager {

    private final TreeMap<Integer,ComplexColor> layers = new TreeMap<>();

    public void add(ComplexColor guy, int zIndex) {
        layers.put(zIndex, guy);
    }

    public void remove(int zIndex) {
        layers.remove(zIndex);
    }

    public ComplexColor getCurrentColor() {
        return layers.isEmpty() ? null : layers.lastEntry().getValue();
    }

    public Integer getCurrentZ() {
        return layers.isEmpty() ? null : layers.lastKey();
    }

    public boolean isTop(int zIndex) {
        return !layers.isEmpty() && layers.lastKey() == zIndex;
    }


} //ok