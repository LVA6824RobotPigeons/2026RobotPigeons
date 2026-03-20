package frc.robot.subsystems.led8.Colors;

import com.ctre.phoenix6.signals.RGBWColor;

public class ColorSequence {

    public ComplexColor complexColor = null;

    public double frequency; // MS
    public int index = 0;
    public RGBWColor[] colors;

    public ColorSequence(RGBWColor[] colors, double frequency) {
        this.frequency = frequency;
        this.colors = colors;
    }

    public void setColor(RGBWColor color) {
        if(complexColor != null) complexColor.currentColor = color;
    }

    public void tick(int refreshRate) {

        setColor(colors[index]);
        if(System.currentTimeMillis()%Math.max(refreshRate,frequency)<=refreshRate) index++;
        if(index == colors.length) index = 0;

    }

}
