package frc.robot.subsystems.led8.Colors;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.subsystems.led8.LED8;

public class ColorSolid {

    private final RGBWColor config_color;

    public ColorSolid(RGBWColor color) {
        this.config_color = color;
    }

    public ComplexColor complexColor = null;

    public void setColor(RGBWColor color, LED8 led8) {
        led8.candle.setControl(
                new SolidColor(
                        led8.startIndex, led8.startIndex+led8.amountOfLeds-1
                )
                .withColor(color)
        );
    }

    public void tick(int refreshRate, LED8 led8) {
        setColor(config_color,led8);
    }

}
