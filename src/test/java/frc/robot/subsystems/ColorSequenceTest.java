package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.signals.RGBWColor;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ColorSequenceTest {
    private static final RGBWColor RED = new RGBWColor(255, 0, 0);
    private static final RGBWColor BLUE = new RGBWColor(0, 0, 255);

    @BeforeEach
    void clearRegisteredSequencesBefore() {
        ColorSequence.colorSequences.clear();
    }

    @AfterEach
    void clearRegisteredSequencesAfter() {
        ColorSequence.colorSequences.clear();
    }

    // make sure we dont get any trash

    @Test
    void emptyPaletteDoesNotEmitColors() {
        RecordingSink sink = new RecordingSink();
        ColorSequence sequence = new ColorSequence(sink::record, new RGBWColor[] {}, 400, 2);

        sequence.tick(20);

        assertTrue(sink.frames.isEmpty());
        // if fail, big concern. so dont. :)
    }

    @Test
    void speedRepresentsStepIntervalMs() {
        RecordingSink sink = new RecordingSink();
        ColorSequence sequence = new ColorSequence(sink::record, new RGBWColor[] {RED, BLUE}, 400, 1);

        for (int i = 0; i < 20; i++) {
            sequence.tick(20);
        }
        sequence.tick(20);

        assertEquals(21, sink.frames.size());
        assertFrame(RED, 1, sink.frames.get(0));
        assertFrame(RED, 1, sink.frames.get(19));
        assertFrame(BLUE, 1, sink.frames.get(20));
        // if this fails here, then our interval timing is wrong. major.
    }

    @Test
    void speedIsClampedToAtLeastOneMillisecond() {
        RecordingSink sink = new RecordingSink();
        ColorSequence sequence = new ColorSequence(sink::record, new RGBWColor[] {RED, BLUE}, 0, 3);

        sequence.tick(1);
        sequence.tick(1);
        sequence.tick(1);

        assertEquals(3, sink.frames.size());
        assertFrame(RED, 3, sink.frames.get(0));
        assertFrame(BLUE, 3, sink.frames.get(1));
        assertFrame(RED, 3, sink.frames.get(2));
        // the code should be pretty self explanitory i mean like
    }

    @Test
    void processTicksAllRegisteredSequences() {
        RecordingSink firstSink = new RecordingSink();
        RecordingSink secondSink = new RecordingSink();

        new ColorSequence(firstSink::record, new RGBWColor[] {RED}, 100, 1);
        new ColorSequence(secondSink::record, new RGBWColor[] {BLUE}, 100, 4);

        ColorSequence.process(20);

        assertEquals(1, firstSink.frames.size());
        assertEquals(1, secondSink.frames.size());
        assertFrame(RED, 1, firstSink.frames.get(0));
        assertFrame(BLUE, 4, secondSink.frames.get(0));
        // make sure tick is doing its job.
    }

    @Test
    void stopUnregistersSequence() {
        RecordingSink sink = new RecordingSink();
        ColorSequence sequence = new ColorSequence(sink::record, new RGBWColor[] {RED}, 100, 1);

        assertTrue(ColorSequence.colorSequences.contains(sequence));
        sequence.stop();

        assertFalse(ColorSequence.colorSequences.contains(sequence));
        // test unregistering.
    }

    private static void assertFrame(RGBWColor expectedColor, int expectedZ, Frame frame) {
        assertEquals(expectedColor.Red, frame.color.Red);
        assertEquals(expectedColor.Green, frame.color.Green);
        assertEquals(expectedColor.Blue, frame.color.Blue);
        assertEquals(expectedZ, frame.zIndex);
    }

    private static class RecordingSink {
        private final List<Frame> frames = new ArrayList<>();

        void record(RGBWColor color, int zIndex) {
            frames.add(new Frame(color, zIndex));
        }
    }

    private static class Frame {
        private final RGBWColor color;
        private final int zIndex;

        Frame(RGBWColor color, int zIndex) {
            this.color = color;
            this.zIndex = zIndex;
        }
    }
    // for testing purposeses
}
