package frc.robot.subsystems.led8.Colors;

import com.ctre.phoenix6.signals.RGBWColor;

public class ComplexColor {

    public enum ColorTypes {
        Solid,
        Animation,
        Sequence,
    }

    public RGBWColor currentColor;
    public ColorTypes type;
    private ColorSequence colorSequence;
    private ColorSolid colorSolid;
    private ColorAnimation colorAnimation;

    public ComplexColor(ColorAnimation colorAnimation) {
        type = ColorTypes.Animation;
        colorAnimation.complexColor = this;
        this.colorAnimation = colorAnimation;
    }
    public ComplexColor(frc.robot.led.led8.Colors.ColorSolid colorSolid) {
        type = ColorTypes.Solid;
        colorSolid.complexColor = this;
        this.colorSolid = colorSolid;
    }
    public ComplexColor(frc.robot.led.led8.Colors.ColorSequence colorSequence) {
        type = ColorTypes.Sequence;
        colorSequence.complexColor = this;
        this.colorSequence = colorSequence;
    }

    public void tick(int refreshRate) {

        switch(type) {
            case Solid:
                colorSolid.tick(refreshRate); break;
            case Animation:
                colorAnimation.tick(refreshRate); break;
            case Sequence:
                colorSequence.tick(refreshRate); break;
        }

    }

}
