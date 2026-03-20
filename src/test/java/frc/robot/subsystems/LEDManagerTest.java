package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.signals.RGBWColor;
import org.junit.jupiter.api.Test;

class LEDManagerTest {
    // here we see my test suite. i am test thing . :thumb:

    @Test
    void emptyManagerHasNoTopLayer() {
        LEDManager manager = new LEDManager();

        assertNull(manager.getCurrentColor());
        assertNull(manager.getCurrentZ());
        assertFalse(manager.isTop(0));
        // first, we create a null manager. then, we check these vals. they dont make sense because its null. therefore, if we get anything else we FAIL!!!!
    }

    @Test
    void highestZLayerWins() {
        LEDManager manager = new LEDManager();
        RGBWColor low = new RGBWColor(10, 20, 30);
        RGBWColor high = new RGBWColor(40, 50, 60);

        manager.set(low, 1);
        manager.set(high, 4);

        assertEquals(4, manager.getCurrentZ());
        assertSameColor(high, manager.getCurrentColor());
        assertTrue(manager.isTop(4));
        assertFalse(manager.isTop(1));
        // now, we have 2 guy. we set high there, so we should get high. if not... idk fail ig
    }

    @Test
    void settingExistingZReplacesColor() {
        LEDManager manager = new LEDManager();
        RGBWColor first = new RGBWColor(1, 2, 3);
        RGBWColor replacement = new RGBWColor(7, 8, 9);

        manager.set(first, 2);
        manager.set(replacement, 2);

        assertEquals(2, manager.getCurrentZ());
        assertSameColor(replacement, manager.getCurrentColor());
        // this is a really crazy one. here, we replace it. simple as.
    }

    @Test
    void removeUsesBothColorAndZ() {
        LEDManager manager = new LEDManager();
        RGBWColor kept = new RGBWColor(9, 9, 9);
        RGBWColor wrong = new RGBWColor(1, 1, 1);

        manager.set(kept, 3);
        manager.remove(3);

        assertEquals(3, manager.getCurrentZ());
        assertSameColor(kept, manager.getCurrentColor());
        // theres only so much i can comment on these. test to see if yeah
    }

    @Test
    void removingTopLayerFallsBackToNextLayer() {
        LEDManager manager = new LEDManager();
        RGBWColor base = new RGBWColor(2, 2, 2);
        RGBWColor top = new RGBWColor(8, 8, 8);

        manager.set(base, 1);
        manager.set(top, 5);
        manager.remove(top, 5);

        assertEquals(1, manager.getCurrentZ());
        assertSameColor(base, manager.getCurrentColor());
        // the code should be pretty self explanitory i mean like
    }

    private static void assertSameColor(RGBWColor expected, RGBWColor actual) {
        assertEquals(expected.Red, actual.Red);
        assertEquals(expected.Green, actual.Green);
        assertEquals(expected.Blue, actual.Blue);
        // :o
    }
}
