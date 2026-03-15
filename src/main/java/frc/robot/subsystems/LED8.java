package frc.robot.subsystems;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import static frc.robot.Constants.LEDs.kNumberOfLights;
import static frc.robot.Constants.LEDs.kStartLED;

public class LED8 {

    private static final RGBWColor kOff = new RGBWColor(0,0,0);
    private final LEDManager manager;
    private final CANdle candle;
    public LED8(CANdle candle) {
        this.manager = new LEDManager();
        this.candle = candle;
        setColor(kOff,0);
    } // Sets colour to black (off)
    public LED8 setColor(RGBWColor color,int zIndex) {
        manager.set(color,zIndex);
        return this;
    }

    public ColorSequence colorSequence;
    public void setColor(RGBWColor[] sequence, long speed, int zIndex) {
        if(this.colorSequence != null) this.colorSequence.stop();
        this.colorSequence = new ColorSequence(this,sequence,speed,zIndex);
    }
    public void removeColor(int zIndex) {
        if(manager.getCurrentZ() == zIndex) manager.remove(manager.getCurrentColor(),manager.getCurrentZ());
    }
    public void process(int refreshRate) {

        colorSequence.tick(refreshRate);
        RGBWColor current = resolveColorOrOff(manager.getCurrentColor());
        candle.setControl(
                new SolidColor(kStartLED, kNumberOfLights-1+kStartLED).withColor(
                        new RGBWColor(
                                current.Red,
                                current.Green,
                                current.Blue
                        )
                )
        );

    }

    static RGBWColor resolveColorOrOff(RGBWColor color) {
        return color == null ? kOff : color;
    }

}
