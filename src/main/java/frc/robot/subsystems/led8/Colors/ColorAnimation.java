package frc.robot.subsystems.led8.Colors;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.subsystems.led8.LED8;

public class ColorAnimation {

    public ComplexColor complexColor = null;

    private enum Animations {
            Fire,
            Larson,
            ColorFlow,
            Rainbow,
            RgbFade,
            SingleFade,
            Strobe,
            Twinkle,
            TwinkleOff

    }

    Animations type;
    FireAnimation fire;
    LarsonAnimation larson;
    ColorFlowAnimation colorFlow;
    RainbowAnimation rainbow;
    RgbFadeAnimation rgbFade;
    SingleFadeAnimation singleFade;
    StrobeAnimation strobe;
    TwinkleAnimation twinkle;
    TwinkleOffAnimation twinkleOff;

    public ColorAnimation(FireAnimation fireAnimation) {
        fire = fireAnimation;
        type = Animations.Fire;
    }
    public ColorAnimation(LarsonAnimation larsonAnimation) {
        larson = larsonAnimation;
        type = Animations.Larson;
    }
    public ColorAnimation(ColorFlowAnimation colorFlowAnimation) {
        colorFlow = colorFlowAnimation;
        type = Animations.ColorFlow;
    }
    public ColorAnimation(RainbowAnimation rainbowAnimation) {
        rainbow = rainbowAnimation;
        type = Animations.Rainbow;
    }
    public ColorAnimation(RgbFadeAnimation rgbFadeAnimation) {
        rgbFade = rgbFadeAnimation;
        type = Animations.RgbFade;
    }
    public ColorAnimation(SingleFadeAnimation singleFadeAnimation) {
        singleFade = singleFadeAnimation;
        type = Animations.SingleFade;
    }
    public ColorAnimation(StrobeAnimation strobeAnimation) {
        strobe = strobeAnimation;
        type = Animations.Strobe;
    }
    public ColorAnimation(TwinkleAnimation twinkleAnimation) {
        twinkle = twinkleAnimation;
        type = Animations.Twinkle;
    }
    public ColorAnimation(TwinkleOffAnimation twinkleOffAnimation) {
        twinkleOff = twinkleOffAnimation;
        type = Animations.TwinkleOff;
    }

    public void setAnimation(FireAnimation fireAnimation, LED8 led8) {
        led8.candle.setControl(
                fireAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(LarsonAnimation larsonAnimation, LED8 led8) {
        led8.candle.setControl(
                larsonAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(ColorFlowAnimation colorFlowAnimation, LED8 led8) {
        led8.candle.setControl(
                colorFlowAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(RainbowAnimation rainbowAnimation, LED8 led8) {
        led8.candle.setControl(
                rainbowAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(RgbFadeAnimation rgbFadeAnimation, LED8 led8) {
        led8.candle.setControl(
                rgbFadeAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(SingleFadeAnimation singleFadeAnimation, LED8 led8) {
        led8.candle.setControl(
                singleFadeAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(StrobeAnimation strobeAnimation, LED8 led8) {
        led8.candle.setControl(
                strobeAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(TwinkleAnimation twinkleAnimation, LED8 led8) {
        led8.candle.setControl(
                twinkleAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }
    public void setAnimation(TwinkleOffAnimation twinkleOffAnimation, LED8 led8) {
        led8.candle.setControl(
                twinkleOffAnimation
                        .withLEDStartIndex(led8.startIndex)
                        .withLEDEndIndex(led8.startIndex+led8.amountOfLeds-1)
        );
    }

    public void tick(int refreshRate, LED8 led8) {

        switch(type) {
            case Fire -> setAnimation(fire,led8);
            case Larson -> setAnimation(larson,led8);
            case ColorFlow -> setAnimation(colorFlow,led8);
            case Rainbow -> setAnimation(rainbow,led8);
            case RgbFade -> setAnimation(rgbFade,led8);
            case SingleFade -> setAnimation(singleFade,led8);
            case Strobe -> setAnimation(strobe,led8);
            case Twinkle -> setAnimation(twinkle,led8);
            case TwinkleOff -> setAnimation(twinkleOff,led8);
            default -> led8.candle.setControl(new SolidColor(led8.startIndex,led8.startIndex+led8.amountOfLeds-1).withColor(new RGBWColor(0,0,0)));
        }

    }

}
