// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class LEDs {
        public static final int kStartLED = 0;
        public static final int kNumberOfLights = 8;
        public static RGBWColor kWhite = new RGBWColor(255,255,255);
        public static RGBWColor kRed = new RGBWColor(255,0,0);
        public static RGBWColor kYellow = new RGBWColor(255,255,0);
        public static RGBWColor kGreen = new RGBWColor(0,255,255);
        public static RGBWColor kCyan = new RGBWColor(0,255,255);
        public static RGBWColor kBlue = new RGBWColor(0,0,255);
        public static RGBWColor kMichenta = new RGBWColor(255,0,255);
        public static RGBWColor kBlack = new RGBWColor(0,0,0);

    }
}
