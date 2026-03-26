package frc.robot.subsystems;

/**
 * Centralized hooks for robot-state LED cues.
 *
 * These methods are intentionally disabled because can not done :( so sad..
 */
public class LED8Implimentation {

    public static void robotStart() {
//        frc.robot.Ports.kStrip.addColor(new ComplexColor(new ColorSolid(new RGBWColor(50,50,50))),-1);
    }
    public static void teleopMode() {
//        frc.robot.Ports.kStrip.addColor(new ComplexColor(new ColorAnimation(new LarsonAnimation(0,0).withColor(new RGBWColor(0,0,50)).withFrameRate(400))),1);
    }
    public static void teleopOff() {
//        frc.robot.Ports.kStrip.removeColor(1);
    }
    public static void autoMode() {
//        frc.robot.Ports.kStrip.addColor(new ComplexColor(new ColorAnimation(new LarsonAnimation(0,0).withColor(new RGBWColor(0,0,50)).withFrameRate(400))),0);
    }
    public static void autoOff() {
//        frc.robot.Ports.kStrip.removeColor(0);
    }

    public static void intakeOn() {
//        Ports.kLights.addColor(new ComplexColor(
//                new ColorSequence(new RGBWColor[] {
//                        Constants.LEDs.kGreen,
//                        Constants.LEDs.kWhite
//                },
//                100)),10
//        );
    }
    public static void intakeOff() {
//        Ports.kLights.removeColor(10);
    }

    public static void feedOn() {
//        Ports.kLights.addColor(new ComplexColor(
//                new ColorSequence(new RGBWColor[] {
//                        Constants.LEDs.kBlue,
//                        Constants.LEDs.kMichenta,
//                },
//                100)),15
//        );
    }
    public static void feedOff() {
//        Ports.kLights.removeColor(15);
    }
}
