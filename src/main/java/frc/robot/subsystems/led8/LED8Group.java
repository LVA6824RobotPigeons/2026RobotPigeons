package frc.robot.subsystems.led8;

import frc.robot.subsystems.led8.Colors.ComplexColor;

import java.util.ArrayList;

public class LED8Group {

    public ArrayList<LED8> leds = new ArrayList<>();

    public LED8Group() {}

    public LED8Group add(LED8 led) {
        leds.add(led);
        return this;
    }

    public void addColor(ComplexColor complexColor, int zIndex) {
        leds.forEach((i) -> i.addColor(complexColor,zIndex));
    }
    public void removeColor(int zIndex) {
        leds.forEach((i) -> i.removeColor(zIndex));
    }

}
