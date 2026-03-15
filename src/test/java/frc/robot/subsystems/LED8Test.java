package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.ctre.phoenix6.signals.RGBWColor;
import org.junit.jupiter.api.Test;

class LED8Test {

    @Test
    void resolveColorOrOffReturnsBlackForNull() {
        RGBWColor resolved = LED8.resolveColorOrOff(null);

        assertEquals(0, resolved.Red);
        assertEquals(0, resolved.Green);
        assertEquals(0, resolved.Blue);
        // the code should be pretty self explanitory i mean like
    }

    @Test
    void resolveColorOrOffReturnsOriginalColorWhenPresent() {
        RGBWColor input = new RGBWColor(3, 4, 5);

        RGBWColor resolved = LED8.resolveColorOrOff(input);

        assertSame(input, resolved);
        // the code should be pretty self explanitory i mean like
    }
}
