package frc.robot.commands.auto.bcnp;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.nio.charset.StandardCharsets;

import org.junit.jupiter.api.Test;

class BcnpSchemaHashTest {
    @Test
    void canonicalizeForBcnp_ignoresFormattingAndDescriptions() {
        final byte[] schemaA = (
            "{"
                + "\"version\":\"3.3\","
                + "\"messages\":["
                + "{\"id\":2,\"name\":\"B\",\"description\":\"keep out\",\"fields\":[{\"name\":\"f\",\"type\":\"uint16\"}]},"
                + "{\"id\":1,\"name\":\"A\",\"fields\":[{\"name\":\"x\",\"type\":\"float32\",\"scale\":1000}]}"
                + "],"
                + "\"description\":\"noise\""
                + "}"
        ).getBytes(StandardCharsets.UTF_8);

        final byte[] schemaB = (
            "{\n"
                + "  \"$schema\": \"bcnp_schema.json\",\n"
                + "  \"messages\": [\n"
                + "    {\"name\":\"A\", \"id\":1, \"fields\":[{\"scale\":1000, \"type\":\"float32\", \"name\":\"x\"}]},\n"
                + "    {\"name\":\"B\", \"id\":2, \"fields\":[{\"type\":\"uint16\", \"name\":\"f\"}]}\n"
                + "  ],\n"
                + "  \"version\": \"3.3\"\n"
                + "}\n"
        ).getBytes(StandardCharsets.UTF_8);

        assertArrayEquals(
            BcnpSchemaHash.canonicalizeForBcnp(schemaA),
            BcnpSchemaHash.canonicalizeForBcnp(schemaB)
        );
    }

    @Test
    void canonicalizeForBcnp_outputsExpectedStableJson() {
        final byte[] schema = (
            "{"
                + "\"version\":\"3.3\","
                + "\"messages\":["
                + "{\"id\":2,\"name\":\"B\",\"fields\":[{\"name\":\"f\",\"type\":\"uint16\"}]},"
                + "{\"id\":1,\"name\":\"A\",\"fields\":[{\"name\":\"x\",\"type\":\"int32\"}]}"
                + "]"
                + "}"
        ).getBytes(StandardCharsets.UTF_8);

        final String canonical = new String(BcnpSchemaHash.canonicalizeForBcnp(schema), StandardCharsets.UTF_8);
        assertEquals(
            "{\"messages\":[{\"fields\":[{\"name\":\"x\",\"type\":\"int32\"}],\"id\":1,\"name\":\"A\"},"
                + "{\"fields\":[{\"name\":\"f\",\"type\":\"uint16\"}],\"id\":2,\"name\":\"B\"}],\"version\":\"3.3\"}",
            canonical
        );
    }
}
