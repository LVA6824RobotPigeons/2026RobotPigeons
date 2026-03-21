package frc.robot.subsystems.led8.Colors;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.subsystems.led8.LED8;

public class ColorSequence {

    public ComplexColor complexColor = null;

    public double frequency; // MS
    public int index = 0;
    public RGBWColor[] colors;

    public ColorSequence(RGBWColor[] colors, double frequency) {
        this.frequency = frequency;
        this.colors = colors;
    }

    public void setColor(RGBWColor color, LED8 led8) {
        led8.candle.setControl(
                new SolidColor(
                        led8.startIndex, led8.startIndex+led8.amountOfLeds-1
                )
                .withColor(color)
        );
    }

    public void tick(int refreshRate, LED8 led8) {

        setColor(colors[index],led8);
        if(System.currentTimeMillis()%Math.max(refreshRate,frequency)<=refreshRate) index++;
        if(index == colors.length) index = 0;

    }

}
