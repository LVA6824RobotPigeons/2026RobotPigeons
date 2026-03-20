package frc.robot.subsystems.led8.Colors;

import com.ctre.phoenix6.signals.RGBWColor;

public class ColorAnimation {

    public ComplexColor complexColor = null;

    public void setColor(RGBWColor color) {
        if(complexColor != null) complexColor.currentColor = color;
    }

    public void tick(int refreshRate) {}

}
