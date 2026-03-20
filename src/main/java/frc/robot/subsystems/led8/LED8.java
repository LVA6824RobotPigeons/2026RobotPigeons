package frc.robot.subsystems.led8;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.subsystems.led8.Colors.ComplexColor;

import static frc.robot.Constants.LEDs.kNumberOfLights;
import static frc.robot.Constants.LEDs.kStartLED;

public class LED8 {

    public CANdle candle;
    public LEDManager manager;

    public LED8(CANdle candle) {
        this.candle = candle;
        manager = new LEDManager();
    }

    public LED8 setColor(ComplexColor complexColor, int zIndex) {
        manager.add(complexColor,zIndex);
        return this;
    }
    public void removeColor(int zIndex) {
        manager.remove(zIndex);
    }

    public void process(int refreshRate) {

        ComplexColor color = manager.getCurrentColor();
        if(color != null) color.tick(refreshRate);
        overrideColor(color == null ? new RGBWColor(0,0,0) : color.currentColor);

    }

    public void overrideColor(RGBWColor color) {
        candle.setControl(
                new SolidColor(kStartLED, kNumberOfLights-1+kStartLED).withColor(
                        new RGBWColor(
                                color.Red,
                                color.Green,
                                color.Blue
                        )
                )
        );
    }



}
