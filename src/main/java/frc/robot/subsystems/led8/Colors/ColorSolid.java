package frc.robot.subsystems.led8.Colors;

import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.led.led8.ComplexColor;

public class ColorSolid {

    private final RGBWColor config_color;

    public ColorSolid(RGBWColor color) {
        this.config_color = color;
    }

    public ComplexColor complexColor = null;

    public void setColor(RGBWColor color) {
        if(complexColor != null) complexColor.currentColor = color;
    }

    public void tick(int refreshRate) {
        setColor(config_color);
    }

}
