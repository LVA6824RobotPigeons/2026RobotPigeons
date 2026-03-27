package frc.robot.commands.auto.bcnp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.junit.jupiter.api.Test;

class BcnpSchemaContractTest {
    private static final Path kSchemaPath = Path.of("src/main/deploy/bcnp/messages.json");
    private static final Pattern kMessagePattern = Pattern.compile(
        "\\{\\s*\"id\"\\s*:\\s*(\\d+)\\s*,\\s*\"name\"\\s*:\\s*\"([^\"]+)\"[\\s\\S]*?\"fields\"\\s*:\\s*\\[(.*?)\\]\\s*\\}",
        Pattern.DOTALL
    );
    private static final Pattern kFieldTypePattern = Pattern.compile("\"type\"\\s*:\\s*\"([^\"]+)\"");

    @Test
    void schemaIds_matchProtocolConstants() throws IOException {
        final Map<String, SchemaMessage> schema = loadSchemaMessages();

        assertEquals(BcnpAutoProtocol.MSG_AUTO_PLAN_REQUEST, schema.get("AutoPlanRequest").id());
        assertEquals(BcnpAutoProtocol.MSG_AUTO_PLAN_RESPONSE, schema.get("AutoPlanResponse").id());
        assertEquals(BcnpAutoProtocol.MSG_AUTO_PHASE_COMMAND, schema.get("AutoPhaseCommand").id());
        assertEquals(BcnpAutoProtocol.MSG_AUTO_PHASE_ACK, schema.get("AutoPhaseAck").id());
        assertEquals(BcnpAutoProtocol.MSG_AUTO_HEARTBEAT, schema.get("AutoHeartbeat").id());
        assertEquals(BcnpAutoProtocol.MSG_AUTO_ABORT, schema.get("AutoAbort").id());
        assertEquals(BcnpAutoProtocol.MSG_AUTO_TELEMETRY, schema.get("AutoTelemetry").id());
    }

    @Test
    void schemaWireSizes_matchProtocolConstants() throws IOException {
        final Map<String, SchemaMessage> schema = loadSchemaMessages();

        assertEquals(BcnpAutoProtocol.WIRE_AUTO_PLAN_REQUEST, schema.get("AutoPlanRequest").wireSizeBytes());
        assertEquals(BcnpAutoProtocol.WIRE_AUTO_PLAN_RESPONSE, schema.get("AutoPlanResponse").wireSizeBytes());
        assertEquals(BcnpAutoProtocol.WIRE_AUTO_PHASE_COMMAND, schema.get("AutoPhaseCommand").wireSizeBytes());
        assertEquals(BcnpAutoProtocol.WIRE_AUTO_PHASE_ACK, schema.get("AutoPhaseAck").wireSizeBytes());
        assertEquals(BcnpAutoProtocol.WIRE_AUTO_HEARTBEAT, schema.get("AutoHeartbeat").wireSizeBytes());
        assertEquals(BcnpAutoProtocol.WIRE_AUTO_ABORT, schema.get("AutoAbort").wireSizeBytes());
        assertEquals(BcnpAutoProtocol.WIRE_AUTO_TELEMETRY, schema.get("AutoTelemetry").wireSizeBytes());
    }

    private static Map<String, SchemaMessage> loadSchemaMessages() throws IOException {
        final String raw = Files.readString(kSchemaPath);
        final Matcher matcher = kMessagePattern.matcher(raw);
        final Map<String, SchemaMessage> byName = new HashMap<>();

        while (matcher.find()) {
            final int id = Integer.parseInt(matcher.group(1));
            final String name = matcher.group(2);
            final String fieldsBlob = matcher.group(3);
            byName.put(name, new SchemaMessage(id, computeWireSize(fieldsBlob)));
        }

        assertNotNull(byName.get("AutoPlanRequest"), "Schema missing AutoPlanRequest");
        assertNotNull(byName.get("AutoPlanResponse"), "Schema missing AutoPlanResponse");
        assertNotNull(byName.get("AutoPhaseCommand"), "Schema missing AutoPhaseCommand");
        assertNotNull(byName.get("AutoPhaseAck"), "Schema missing AutoPhaseAck");
        assertNotNull(byName.get("AutoHeartbeat"), "Schema missing AutoHeartbeat");
        assertNotNull(byName.get("AutoAbort"), "Schema missing AutoAbort");
        assertNotNull(byName.get("AutoTelemetry"), "Schema missing AutoTelemetry");
        return byName;
    }

    private static int computeWireSize(String fieldsBlob) {
        final Matcher fieldTypeMatcher = kFieldTypePattern.matcher(fieldsBlob);
        int total = 0;
        while (fieldTypeMatcher.find()) {
            total += wireSizeForType(fieldTypeMatcher.group(1));
        }
        return total;
    }

    private static int wireSizeForType(String type) {
        return switch (type) {
            case "int8", "uint8" -> 1;
            case "int16", "uint16" -> 2;
            case "int32", "uint32", "float32" -> 4;
            case "int64", "uint64", "float64" -> 8;
            default -> throw new IllegalArgumentException("Unsupported schema type: " + type);
        };
    }

    private record SchemaMessage(int id, int wireSizeBytes) {}
}
