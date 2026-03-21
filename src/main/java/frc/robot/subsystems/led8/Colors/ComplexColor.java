package frc.robot.subsystems.led8.Colors;

import frc.robot.subsystems.led8.LED8;

public class ComplexColor {

    public enum ColorTypes {
        Solid,
        Animation,
        Sequence,
    }

    public ColorTypes type;
    private ColorSequence colorSequence;
    private ColorSolid colorSolid;
    private ColorAnimation colorAnimation;

    public ComplexColor(ColorAnimation colorAnimation) {
        type = ColorTypes.Animation;
        colorAnimation.complexColor = this;
        this.colorAnimation = colorAnimation;
    }
    public ComplexColor(ColorSolid colorSolid) {
        type = ColorTypes.Solid;
        colorSolid.complexColor = this;
        this.colorSolid = colorSolid;
    }
    public ComplexColor(ColorSequence colorSequence) {
        type = ColorTypes.Sequence;
        colorSequence.complexColor = this;
        this.colorSequence = colorSequence;
    }

    public void tick(int refreshRate, LED8 led8) {

        switch(type) {
            case Solid:
                colorSolid.tick(refreshRate,led8); break;
            case Animation:
                colorAnimation.tick(refreshRate,led8); break;
            case Sequence:
                colorSequence.tick(refreshRate,led8); break;
        }

    }

}
