package frc.robot.subsystems.led8;

import frc.robot.subsystems.led8.Colors.ComplexColor;

import java.util.TreeMap;


public class LEDManager {

    private final TreeMap<Integer,ComplexColor> layers = new TreeMap<>();

    public void add(ComplexColor guy, int zIndex) {
        layers.put(zIndex, guy);
    }

    public boolean has(ComplexColor guy, int zIndex) {
        if(layers.get(zIndex) == null) return false;
        else return layers.get(zIndex) == guy;
    }

    public ComplexColor remove(int zIndex) { // Returns old value
        return layers.remove(zIndex);
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