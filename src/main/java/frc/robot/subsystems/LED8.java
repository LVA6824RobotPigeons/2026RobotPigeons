package frc.robot.subsystems;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import org.ejml.dense.row.linsol.qr.SolveNullSpaceQR_DDRM;

import static frc.robot.Constants.LEDs.kNumberOfLights;
import static frc.robot.Constants.LEDs.kStartLED;

public class LED8 {
    public static RGBWColor kWhite = new RGBWColor(255,255,255);
    public static RGBWColor kRed = new RGBWColor(255,0,0);
    public static RGBWColor kYellow = new RGBWColor(255,255,0);
    public static RGBWColor kGreen = new RGBWColor(0,255,255);
    public static RGBWColor kCyan = new RGBWColor(0,255,255);
    public static RGBWColor kBlue = new RGBWColor(0,0,255);
    public static RGBWColor kMagenta = new RGBWColor(255,0,255);
    public static RGBWColor kBlack = new RGBWColor(0,0,0);

    private final CANdle candle;

    public LED8(CANdle candle) {
        this.candle = candle;
        setColor(255,255,255);
    }

    public LED8 setColor(int r, int g, int b) {
        return setColor(new RGBWColor(r,g,b));
    }
    public LED8 setColor(RGBWColor color) {
        candle.setControl(
                new SolidColor(kStartLED, kNumberOfLights-1+kStartLED).withColor(
                        new RGBWColor(
                                color.Red,
                                color.Green,
                                color.Blue
                        )
                )
        );
        return this;
    }




}
/*
Made by Gabe A, Miche
 */