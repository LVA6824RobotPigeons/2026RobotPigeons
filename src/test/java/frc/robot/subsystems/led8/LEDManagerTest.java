package frc.robot.subsystems.led8;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;

import frc.robot.subsystems.led8.Colors.ComplexColor;
import org.junit.jupiter.api.Test;

class LEDManagerTest {
    @Test
    void addAndRemove_tracksTopLayerByHighestZIndex() {
        final LEDManager manager = new LEDManager();
        final ComplexColor low = mock(ComplexColor.class);
        final ComplexColor high = mock(ComplexColor.class);

        manager.add(low, 5);
        manager.add(high, 10);

        assertEquals(high, manager.getCurrentColor());
        assertEquals(10, manager.getCurrentZ());
        assertTrue(manager.isTop(10));

        final ComplexColor removed = manager.remove(10);
        assertEquals(high, removed);
        assertEquals(low, manager.getCurrentColor());
        assertEquals(5, manager.getCurrentZ());
    }

    @Test
    void has_usesObjectIdentityAtSameLayer() {
        final LEDManager manager = new LEDManager();
        final ComplexColor stored = mock(ComplexColor.class);
        final ComplexColor differentInstance = mock(ComplexColor.class);

        manager.add(stored, 3);

        assertTrue(manager.has(stored, 3));
        assertFalse(manager.has(differentInstance, 3));
    }

    @Test
    void emptyManager_reportsNoTopLayer() {
        final LEDManager manager = new LEDManager();

        assertNull(manager.getCurrentColor());
        assertNull(manager.getCurrentZ());
        assertFalse(manager.isTop(0));
    }
}
