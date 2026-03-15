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
        process();
        return this;
    }

    public ColorSequence colorSequence;
    public void setColorSequence(RGBWColor[] sequence,long speed,int zIndex) {
        if(this.colorSequence != null) this.colorSequence.stop();
        this.colorSequence = new ColorSequence(this,sequence,speed,zIndex);

    }
    /*
    * here i added a kOff constant.
    * if manager is null, we should be off.
    * since we manually manage our states, null is a valid state to be in.
    * but our old code would try to read the R val of null and throw a nullptr error.
    * very bad.
    *
    * so instead, we fall back to black now! :D
     */
    public void process() {

        RGBWColor current = manager.getCurrentColor();
        if (current == null) {
            current = kOff;
        }
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

}
