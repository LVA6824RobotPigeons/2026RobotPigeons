package frc.robot.subsystems.led8;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.swerve.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Ports;

public class ErrorHandler {

    private static final RGBWColor[] kDefaultExceptionAnimation = new RGBWColor[] {
            Constants.LEDs.kRed,
            Constants.LEDs.kBlack
    };
    private static final long kDefaultExceptionAnimationSpeedMs = 125;
    private static final int kDefaultExceptionZIndex = 999;

    private ErrorHandler() {}

    public static void runGuarded (String context, Runnable runabble) {
        try {
            runabble.run();
        } catch (Throwable throwbable) {
            handleThrowable(context, throwbable);
        }
    }

    public static void meow () {
        System.out.println("hi");
    }

    public static void handleThrowable(String context, Throwable throwable) {
        DriverStation.reportError("[ErrorHandler :D] " + context + ": " + throwable.getMessage(), throwable.getStackTrace());
        try {
            //set anim lol
            meow();
        } catch (Throwable ledThrowable) {
            DriverStation.reportError(
                    "[ErrorHandler D:] LED exception animation failed: " + ledThrowable.getMessage(),
                    ledThrowable.getStackTrace()
            );
        } finally {
            DriverStation.reportWarning("meow", false);
        };
    }
    public static void indicateWithColor(String context, Throwable throwable, RGBWColor color, int zIndex) {
        DriverStation.reportError("[ErrorHandler :)] " + context + ": " + throwable.getMessage(), throwable.getStackTrace());
        try {
           // solid
            meow();
        } catch (Throwable ledThrowable) {
            DriverStation.reportError(
                    "[ErrorHandler :c] LED exception color failed: " + ledThrowable.getMessage(),
                    ledThrowable.getStackTrace()
            );
        }
    }

    public static void indicateWithAnimation(String context, Throwable throwable, RGBWColor[] sequence, long speedMs, int zIndex) {
        DriverStation.reportError("[RErrorHandler :b] " + context + ": " + throwable.getMessage(), throwable.getStackTrace());
        try {
            //set anim lol
            meow();
        } catch (Throwable ledThrowable) {
            DriverStation.reportError(
                    "[ErrorHandler :d] LED exception animation failed: " + ledThrowable.getMessage(),
                    ledThrowable.getStackTrace()
            );
        }
    }
}
