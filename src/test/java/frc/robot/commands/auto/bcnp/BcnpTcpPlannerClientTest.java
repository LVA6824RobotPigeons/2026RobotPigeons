package frc.robot.commands.auto.bcnp;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.nio.channels.SocketChannel;

import org.junit.jupiter.api.Test;

class BcnpTcpPlannerClientTest {
    @Test
    void readIncoming_ignoresNotYetConnectedSocket() throws Exception {
        final BcnpTcpPlannerClient client = new BcnpTcpPlannerClient(
                "127.0.0.1",
                5809,
                0x12345678,
                20,
                100,
                500);
        final SocketChannel unconnectedChannel = SocketChannel.open();
        unconnectedChannel.configureBlocking(false);

        setField(client, "channel", unconnectedChannel);

        try {
            final Method readIncoming = client.getClass().getDeclaredMethod("readIncoming", long.class);
            readIncoming.setAccessible(true);
            assertDoesNotThrow(() -> readIncoming.invoke(client, System.currentTimeMillis()));
        } finally {
            client.close();
        }
    }

    private static void setField(Object target, String fieldName, Object value) throws Exception {
        final Field field = target.getClass().getDeclaredField(fieldName);
        field.setAccessible(true);
        field.set(target, value);
    }
}
