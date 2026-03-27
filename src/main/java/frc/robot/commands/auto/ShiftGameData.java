package frc.robot.commands.auto;

// Parsed summary of game data relevant to SHIFT-1 hub inactivity.
public enum ShiftGameData {
    UNKNOWN,
    RED_INACTIVE_SHIFT1,
    BLUE_INACTIVE_SHIFT1;

    // Parses game-specific message from FMS.
    // Current expected formats include single-char tokens (`R`/`B`).
    public static ShiftGameData parse(String gameSpecificMessage) {
        if (gameSpecificMessage == null || gameSpecificMessage.isBlank()) {
            return UNKNOWN;
        }
        final char token = Character.toUpperCase(gameSpecificMessage.trim().charAt(0));
        return switch (token) {
            case 'R' -> RED_INACTIVE_SHIFT1;
            case 'B' -> BLUE_INACTIVE_SHIFT1;
            default -> UNKNOWN;
        };
    }
}
