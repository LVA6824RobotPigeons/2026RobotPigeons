// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.commands.auto.PlannerExecutionMode;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    public static class Autonomous {
        public static final boolean kUseBcnpPlanner = false;
        // LOCAL_ONLY: ignore planner, SHADOW: observe planner, ACTIVE: planner health
        // gates execution.
        public static final PlannerExecutionMode kPlannerExecutionMode = PlannerExecutionMode.SHADOW;
        public static final String kBcnpHost = "10.57.33.6";
        public static final int kBcnpPort = 5800;
        public static final String kBcnpSchemaDeployPath = "bcnp/messages.json";
        public static final int kBcnpSchemaHashFallback = 0x1A2B3C4D;
        public static final long kBcnpConnectRetryMs = 1000;
        public static final long kBcnpHeartbeatPeriodMs = 100;
        public static final long kBcnpHeartbeatTimeoutMs = 350;
        public static final long kBcnpPlanFreshMs = 750;
        public static final int kActivePlanMinConfidencePermille = 450;
    }

    public static class LEDs {
        public static final int kStartLED = 0;
        public static final int kNumberOfLights = 8;
        // Shared palette used by LED8 effect definitions.
        public static RGBWColor kWhite = new RGBWColor(255, 255, 255);
        public static RGBWColor kRed = new RGBWColor(255, 0, 0);
        public static RGBWColor kYellow = new RGBWColor(255, 255, 0);
        public static RGBWColor kGreen = new RGBWColor(0, 255, 0);
        public static RGBWColor kCyan = new RGBWColor(0, 255, 255);
        public static RGBWColor kBlue = new RGBWColor(0, 0, 255);
        public static RGBWColor kMichenta = new RGBWColor(255, 0, 255);
        public static RGBWColor kBlack = new RGBWColor(0, 0, 0);

    }
}
