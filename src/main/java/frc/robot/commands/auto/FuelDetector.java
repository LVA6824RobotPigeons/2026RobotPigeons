package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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

    private static final double kShooterActiveTargetRpm = 1800.0;
    private static final double kShooterActiveMeasuredRpm = 1200.0;
    private static final double kShooterStartupIgnoreSeconds = 0.30;
    private static final double kShooterBaselineAlpha = 0.08;
    private static final double kShooterEntryCurrentDeltaAmps = 5.0;
    private static final double kShooterEntryRpmDrop = 220.0;
    private static final double kShooterExitCurrentDeltaAmps = 1.8;
    private static final double kShooterExitRpmDrop = 80.0;
    private static final double kShooterDebounceSeconds = 0.03;
    private static final double kShooterRearmSeconds = 0.12;

    private final Intake intake;
    private final Feeder feeder;
    private final Shooter shooter;
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

    private boolean shooterBaselineInitialized = false;
    private double shooterBaselineCurrentAmps = 0.0;
    private double shooterBaselineRpm = 0.0;
    private boolean shooterEventActive = false;
    private boolean shooterLastActive = false;
    private double shooterAboveThresholdSince = -1;
    private double shooterBelowThresholdSince = -1;
    private double shooterIgnoreUntilSeconds = 0.0;

    public FuelDetector(Intake intake, Feeder feeder, Shooter shooter) {
        this(intake, feeder, shooter, Timer::getFPGATimestamp);
    }

    FuelDetector(Intake intake, Feeder feeder, Shooter shooter, DoubleSupplier timeSecondsSupplier) {
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.timeSecondsSupplier = timeSecondsSupplier;
    }

    // Call each cycle to update fuel detection state.
    public boolean update() {
        final double now = timeSecondsSupplier.getAsDouble();
        final boolean acquiredFuel = updateIntakeDetection(now);
        updateShooterEjectionDetection(now);
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

        shooterBaselineInitialized = false;
        shooterBaselineCurrentAmps = 0.0;
        shooterBaselineRpm = 0.0;
        shooterEventActive = false;
        shooterLastActive = false;
        shooterAboveThresholdSince = -1;
        shooterBelowThresholdSince = -1;
        shooterIgnoreUntilSeconds = 0.0;
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

    private void updateShooterEjectionDetection(double nowSeconds) {
        final double shooterTargetRpm = shooter.getTargetRpm();
        final double shooterRpm = shooter.getAverageRpm();
        final double shooterCurrentAmps = shooter.getAverageSupplyCurrentAmps();
        final boolean shooterActive = shooterTargetRpm >= kShooterActiveTargetRpm
                && shooterRpm >= kShooterActiveMeasuredRpm;

        if (!shooterBaselineInitialized) {
            shooterBaselineInitialized = true;
            shooterBaselineCurrentAmps = shooterCurrentAmps;
            shooterBaselineRpm = shooterRpm;
            return;
        }

        if (shooterActive && !shooterLastActive) {
            shooterIgnoreUntilSeconds = nowSeconds + kShooterStartupIgnoreSeconds;
        }
        shooterLastActive = shooterActive;

        if (!shooterActive) {
            shooterEventActive = false;
            shooterAboveThresholdSince = -1;
            shooterBelowThresholdSince = -1;
            shooterBaselineCurrentAmps = shooterCurrentAmps;
            shooterBaselineRpm = shooterRpm;
            return;
        }

        if (nowSeconds < shooterIgnoreUntilSeconds) {
            shooterBaselineCurrentAmps = shooterBaselineCurrentAmps + (0.20 * (shooterCurrentAmps - shooterBaselineCurrentAmps));
            shooterBaselineRpm = shooterBaselineRpm + (0.20 * (shooterRpm - shooterBaselineRpm));
            return;
        }

        if (!shooterEventActive) {
            shooterBaselineCurrentAmps = shooterBaselineCurrentAmps
                    + (kShooterBaselineAlpha * (shooterCurrentAmps - shooterBaselineCurrentAmps));
            shooterBaselineRpm = shooterBaselineRpm + (kShooterBaselineAlpha * (shooterRpm - shooterBaselineRpm));

            final double currentDeltaAmps = shooterCurrentAmps - shooterBaselineCurrentAmps;
            final double rpmDrop = shooterBaselineRpm - shooterRpm;
            if (currentDeltaAmps >= kShooterEntryCurrentDeltaAmps || rpmDrop >= kShooterEntryRpmDrop) {
                if (shooterAboveThresholdSince < 0) {
                    shooterAboveThresholdSince = nowSeconds;
                }
                if (nowSeconds - shooterAboveThresholdSince >= kShooterDebounceSeconds) {
                    shooterEventActive = true;
                    shooterBelowThresholdSince = -1;
                }
            } else {
                shooterAboveThresholdSince = -1;
            }
            return;
        }

        final double currentDeltaAmps = shooterCurrentAmps - shooterBaselineCurrentAmps;
        final double rpmDrop = shooterBaselineRpm - shooterRpm;
        if (currentDeltaAmps <= kShooterExitCurrentDeltaAmps && rpmDrop <= kShooterExitRpmDrop) {
            if (shooterBelowThresholdSince < 0) {
                shooterBelowThresholdSince = nowSeconds;
            }
            if (nowSeconds - shooterBelowThresholdSince >= kShooterDebounceSeconds) {
                shooterEventActive = false;
                shooterAboveThresholdSince = -1;
                shooterBelowThresholdSince = -1;
                shooterIgnoreUntilSeconds = nowSeconds + kShooterRearmSeconds;
                if (fuelCount > 0) {
                    fuelCount--;
                }
            }
        } else {
            shooterBelowThresholdSince = -1;
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
