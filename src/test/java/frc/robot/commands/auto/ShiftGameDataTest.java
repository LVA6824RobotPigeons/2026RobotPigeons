package frc.robot.commands.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class ShiftGameDataTest {
    @Test
    void parse_handlesKnownTokens() {
        assertEquals(ShiftGameData.RED_INACTIVE_SHIFT1, ShiftGameData.parse("R"));
        assertEquals(ShiftGameData.BLUE_INACTIVE_SHIFT1, ShiftGameData.parse("b"));
    }

    @Test
    void parse_unknownForEmptyOrUnexpectedValues() {
        assertEquals(ShiftGameData.UNKNOWN, ShiftGameData.parse(""));
        assertEquals(ShiftGameData.UNKNOWN, ShiftGameData.parse("A"));
        assertEquals(ShiftGameData.UNKNOWN, ShiftGameData.parse(null));
    }
}
