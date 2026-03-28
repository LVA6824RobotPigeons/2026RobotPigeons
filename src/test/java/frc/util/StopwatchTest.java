package frc.util;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class StopwatchTest {
    @BeforeAll
    static void initializeHal() {
        HAL.initialize(500, 0);
    }

    @Test
    void elapsedSeconds_isZeroUntilStarted() {
        final Stopwatch stopwatch = new Stopwatch();
        assertEquals(0.0, stopwatch.elapsedSeconds(), 1e-12);
    }

    @Test
    void startAndReset_updateElapsedAsExpected() {
        final Stopwatch stopwatch = new Stopwatch();

        stopwatch.start();
        Timer.delay(0.02);
        final double elapsedAfterStart = stopwatch.elapsedSeconds();
        assertTrue(elapsedAfterStart >= 0.01);

        stopwatch.reset();
        assertEquals(0.0, stopwatch.elapsedSeconds(), 1e-12);
    }

    @Test
    void startIfNotRunning_doesNotRestartAnActiveStopwatch() {
        final Stopwatch stopwatch = new Stopwatch();

        stopwatch.startIfNotRunning();
        Timer.delay(0.01);
        final double firstElapsed = stopwatch.elapsedSeconds();

        stopwatch.startIfNotRunning();
        Timer.delay(0.01);
        final double secondElapsed = stopwatch.elapsedSeconds();

        assertTrue(secondElapsed > firstElapsed);
    }
}
