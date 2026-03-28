package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

// Detects fuel acquisition by monitoring intake roller motor current.
public class FuelDetector {
    private static final int kMaxFuelCount = 3;
    private static final double kIntakeActiveCurrentAmps = 2.0;
    private static final double kStartupIgnoreSeconds = 0.32;
    private static final double kIntakeDebounceSeconds = 0.05;
    private static final double kIntakeExitDeltaAmps = 1.3;
    private static final double kIntakeRearmSeconds = 0.10;
    private static final double kBaselineAlpha = 0.06;
    private static final double kNoiseAlpha = 0.25;
    private static final double kIntakeMinEntryDeltaAmps = 2.4;
    private static final double kIntakeNoiseMultiplier = 3.0;
    private static final double kZone2BallsDeltaAmps = 5.0;
    private static final double kZone3BallsDeltaAmps = 8.5;

    private static final double kFeederActiveRpm = 1200.0;
    private static final double kFeederBaselineAlpha = 0.08;
    private static final double kFeederEntryDeltaAmps = 3.0;
    private static final double kFeederExitDeltaAmps = 1.2;
    private static final double kFeederDebounceSeconds = 0.03;
    private static final double kFeederRearmSeconds = 0.12;

    private final Intake intake;
    private final Feeder feeder;
    private final DoubleSupplier timeSecondsSupplier;

    private int fuelCount = 0;
    private boolean acquisitionEventLatched = false;

    private double lastIntakeCurrentAmps = 0.0;
    private double intakeBaselineAmps = 0.0;
    private double intakeNoiseAmps = 0.0;
    private boolean intakeBaselineInitialized = false;
    private boolean lastIntakeActive = false;
    private boolean intakeEventActive = false;
    private double intakeEventBelowExitSeconds = -1;
    private double intakeEventPeakDeltaAmps = 0.0;
    private double intakeAboveThresholdSince = -1;
    private double intakeIgnoreUntilSeconds = 0.0;

    private boolean feederBaselineInitialized = false;
    private double feederBaselineAmps = 0.0;
    private boolean feederEventActive = false;
    private double feederAboveThresholdSince = -1;
    private double feederBelowThresholdSince = -1;
    private double feederIgnoreUntilSeconds = 0.0;

    public FuelDetector(Intake intake, Feeder feeder) {
        this(intake, feeder, Timer::getFPGATimestamp);
    }

    FuelDetector(Intake intake, Feeder feeder, DoubleSupplier timeSecondsSupplier) {
        this.intake = intake;
        this.feeder = feeder;
        this.timeSecondsSupplier = timeSecondsSupplier;
    }

    // Call each cycle to update fuel detection state.
    public boolean update() {
        final double now = timeSecondsSupplier.getAsDouble();
        final boolean acquiredFuel = updateIntakeDetection(now);
        updateFeederEjectionDetection(now);
        if (acquiredFuel) {
            acquisitionEventLatched = true;
        }
        return acquiredFuel;
    }

    public boolean consumeAcquisitionEvent() {
        if (!acquisitionEventLatched) {
            return false;
        }
        acquisitionEventLatched = false;
        return true;
    }

    // Return true when intake current is in an active, debounced fuel event.
    public boolean hasFuelRaw() {
        return intakeEventActive;
    }

    public int getFuelCount() {
        return fuelCount;
    }

    public void reset() {
        fuelCount = 0;
        acquisitionEventLatched = false;

        lastIntakeCurrentAmps = 0.0;
        intakeBaselineAmps = 0.0;
        intakeNoiseAmps = 0.0;
        intakeBaselineInitialized = false;
        lastIntakeActive = false;
        intakeEventActive = false;
        intakeEventBelowExitSeconds = -1;
        intakeEventPeakDeltaAmps = 0.0;
        intakeAboveThresholdSince = -1;
        intakeIgnoreUntilSeconds = 0.0;

        feederBaselineInitialized = false;
        feederBaselineAmps = 0.0;
        feederEventActive = false;
        feederAboveThresholdSince = -1;
        feederBelowThresholdSince = -1;
        feederIgnoreUntilSeconds = 0.0;
    }

    private boolean updateIntakeDetection(double nowSeconds) {
        final double currentAmps = intake.getRollerCurrent();
        final boolean intakeActive = currentAmps >= kIntakeActiveCurrentAmps;

        if (!intakeBaselineInitialized) {
            intakeBaselineInitialized = true;
            intakeBaselineAmps = currentAmps;
            lastIntakeCurrentAmps = currentAmps;
            return false;
        }

        intakeNoiseAmps = ((1.0 - kNoiseAlpha) * intakeNoiseAmps)
                + (kNoiseAlpha * Math.abs(currentAmps - lastIntakeCurrentAmps));
        lastIntakeCurrentAmps = currentAmps;

        if (intakeActive && !lastIntakeActive) {
            intakeIgnoreUntilSeconds = nowSeconds + kStartupIgnoreSeconds;
        }
        lastIntakeActive = intakeActive;

        if (!intakeActive) {
            intakeEventActive = false;
            intakeAboveThresholdSince = -1;
            intakeEventBelowExitSeconds = -1;
            intakeBaselineAmps = currentAmps;
            return false;
        }

        if (nowSeconds < intakeIgnoreUntilSeconds) {
            intakeEventActive = false;
            intakeAboveThresholdSince = -1;
            intakeEventBelowExitSeconds = -1;
            intakeBaselineAmps = intakeBaselineAmps + (0.30 * (currentAmps - intakeBaselineAmps));
            return false;
        }

        final double dynamicEntryThreshold = Math.max(
                kIntakeMinEntryDeltaAmps,
                kIntakeNoiseMultiplier * intakeNoiseAmps);
        final double deltaAmps = currentAmps - intakeBaselineAmps;

        if (!intakeEventActive) {
            intakeBaselineAmps = intakeBaselineAmps + (kBaselineAlpha * (currentAmps - intakeBaselineAmps));

            if (deltaAmps >= dynamicEntryThreshold) {
                if (intakeAboveThresholdSince < 0) {
                    intakeAboveThresholdSince = nowSeconds;
                }
                if (nowSeconds - intakeAboveThresholdSince >= kIntakeDebounceSeconds) {
                    intakeEventActive = true;
                    intakeEventPeakDeltaAmps = deltaAmps;
                    intakeEventBelowExitSeconds = -1;
                }
            } else {
                intakeAboveThresholdSince = -1;
            }
            return false;
        }

        intakeEventPeakDeltaAmps = Math.max(intakeEventPeakDeltaAmps, deltaAmps);
        if (deltaAmps <= kIntakeExitDeltaAmps) {
            if (intakeEventBelowExitSeconds < 0) {
                intakeEventBelowExitSeconds = nowSeconds;
            }
            if (nowSeconds - intakeEventBelowExitSeconds >= kIntakeDebounceSeconds) {
                intakeEventActive = false;
                intakeAboveThresholdSince = -1;
                intakeEventBelowExitSeconds = -1;
                intakeIgnoreUntilSeconds = nowSeconds + kIntakeRearmSeconds;
                final int increment = classifyIntakeEvent(intakeEventPeakDeltaAmps);
                if (increment > 0) {
                    fuelCount = Math.min(kMaxFuelCount, fuelCount + increment);
                    return true;
                }
            }
        } else {
            intakeEventBelowExitSeconds = -1;
        }

        return false;
    }

    private void updateFeederEjectionDetection(double nowSeconds) {
        final double feederRpm = feeder.getRpm();
        final double feederCurrentAmps = feeder.getSupplyCurrentAmps();
        final boolean feederActive = feederRpm >= kFeederActiveRpm;

        if (!feederBaselineInitialized) {
            feederBaselineInitialized = true;
            feederBaselineAmps = feederCurrentAmps;
            return;
        }

        if (!feederActive) {
            feederEventActive = false;
            feederAboveThresholdSince = -1;
            feederBelowThresholdSince = -1;
            feederBaselineAmps = feederCurrentAmps;
            return;
        }

        if (nowSeconds < feederIgnoreUntilSeconds) {
            feederBaselineAmps = feederBaselineAmps + (0.20 * (feederCurrentAmps - feederBaselineAmps));
            return;
        }

        if (!feederEventActive) {
            feederBaselineAmps = feederBaselineAmps + (kFeederBaselineAlpha * (feederCurrentAmps - feederBaselineAmps));
            final double deltaAmps = feederCurrentAmps - feederBaselineAmps;
            if (deltaAmps >= kFeederEntryDeltaAmps) {
                if (feederAboveThresholdSince < 0) {
                    feederAboveThresholdSince = nowSeconds;
                }
                if (nowSeconds - feederAboveThresholdSince >= kFeederDebounceSeconds) {
                    feederEventActive = true;
                    feederBelowThresholdSince = -1;
                }
            } else {
                feederAboveThresholdSince = -1;
            }
            return;
        }

        final double deltaAmps = feederCurrentAmps - feederBaselineAmps;
        if (deltaAmps <= kFeederExitDeltaAmps) {
            if (feederBelowThresholdSince < 0) {
                feederBelowThresholdSince = nowSeconds;
            }
            if (nowSeconds - feederBelowThresholdSince >= kFeederDebounceSeconds) {
                feederEventActive = false;
                feederAboveThresholdSince = -1;
                feederBelowThresholdSince = -1;
                feederIgnoreUntilSeconds = nowSeconds + kFeederRearmSeconds;
                if (fuelCount > 0) {
                    fuelCount--;
                }
            }
        } else {
            feederBelowThresholdSince = -1;
        }
    }

    private int classifyIntakeEvent(double peakDeltaAmps) {
        if (peakDeltaAmps < kIntakeMinEntryDeltaAmps) {
            return 0;
        }
        if (peakDeltaAmps >= kZone3BallsDeltaAmps) {
            return 3;
        }
        if (peakDeltaAmps >= kZone2BallsDeltaAmps) {
            return 2;
        }
        return 1;
    }
}
