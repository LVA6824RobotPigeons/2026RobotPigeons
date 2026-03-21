package frc.robot.subsystems.led8;

import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.Constants;
import frc.robot.subsystems.led8.Colors.ColorSequence;
import frc.robot.subsystems.led8.Colors.ColorSolid;
import frc.robot.subsystems.led8.Colors.ComplexColor;

public class LED8Implimentation {

    public static void intakeOn() {
        Ports.kCandle.setColor(new ComplexColor(
                new ColorSequence(new RGBWColor[] {
                        Constants.LEDs.kGreen,
                        Constants.LEDs.kWhite
                },
                100)),10
        );
    }
    public static void intakeOff() {
        Ports.kCandle.removeColor(10);
    }

    public static void feedOn() {
        Ports.kCandle.setColor(new ComplexColor(
                new ColorSequence(new RGBWColor[] {
                        Constants.LEDs.kBlue,
                        Constants.LEDs.kMichenta,
                },
                100)),15
        );
    }
    public static void feedOff() {
        Ports.kCandle.removeColor(15);
    }
}
