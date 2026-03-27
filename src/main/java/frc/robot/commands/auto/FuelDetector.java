package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;

// Detects fuel acquisition by monitoring intake roller motor current.
public class FuelDetector {
    private static final double kCurrentThresholdAmps = 15.0;
    private static final double kDebounceDurationSeconds = 0.1;

    private final Intake intake;
    private double spikeStartTimestamp = -1;
    private int fuelCount = 0;
    private boolean lastHadFuel = false;

    public FuelDetector(Intake intake) {
        this.intake = intake;
    }

    // Call each cycle to update fuel detection state.
    public boolean update() {
        final boolean hasFuelNow = hasFuelRaw();
        final boolean risingEdge = hasFuelNow && !lastHadFuel;
        lastHadFuel = hasFuelNow;
        if (risingEdge) {
            fuelCount++;
        }
        return risingEdge;
    }

    // Return true if the roller current has been above threshold
    // for at least the debounce duration.
    public boolean hasFuelRaw() {
        final double current = intake.getRollerCurrent();
        if (current > kCurrentThresholdAmps) {
            if (spikeStartTimestamp < 0) {
                spikeStartTimestamp = Timer.getFPGATimestamp();
            }
            return Timer.getFPGATimestamp() - spikeStartTimestamp > kDebounceDurationSeconds;
        }
        spikeStartTimestamp = -1;
        return false;
    }

    // Return total fuel acquisitions detected this session.
    public int getFuelCount() {
        return fuelCount;
    }

    // Reset the fuel counter (e.g., at start of auto/teleop).
    public void reset() {
        fuelCount = 0;
        spikeStartTimestamp = -1;
        lastHadFuel = false;
    }
}
