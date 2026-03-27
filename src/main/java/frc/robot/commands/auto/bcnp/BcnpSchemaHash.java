package frc.robot.commands.auto.bcnp;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.CRC32;

import edu.wpi.first.wpilibj.Filesystem;

public final class BcnpSchemaHash {
    private static final Pattern kVersionPattern = Pattern.compile("\"version\"\\s*:\\s*\"([^\"]+)\"");
    private static final Pattern kIdPattern = Pattern.compile("\"id\"\\s*:\\s*(\\d+)");
    private static final Pattern kNamePattern = Pattern.compile("\"name\"\\s*:\\s*\"([^\"]+)\"");
    private static final Pattern kTypePattern = Pattern.compile("\"type\"\\s*:\\s*\"([^\"]+)\"");
    private static final Pattern kScalePattern = Pattern.compile("\"scale\"\\s*:\\s*([\\-\\d\\.eE\\+]+)");

    private BcnpSchemaHash() {
    }

    public static int loadOrFallback(String deployRelativePath, int fallbackHash) {
        final Path schemaPath = Filesystem.getDeployDirectory().toPath().resolve(deployRelativePath);
        try {
            final byte[] raw = Files.readAllBytes(schemaPath);
            return crc32(canonicalizeForBcnp(raw));
        } catch (IOException e) {
            return fallbackHash;
        } catch (IllegalArgumentException e) {
            return fallbackHash;
        }
    }

    private static int crc32(byte[] bytes) {
        final CRC32 crc32 = new CRC32();
        crc32.update(bytes, 0, bytes.length);
        return (int) crc32.getValue();
    }

    // Canonicalize
    static byte[] canonicalizeForBcnp(byte[] raw) {
        final String rawJson = new String(raw, StandardCharsets.UTF_8);
        final String version = extractRequired(kVersionPattern, rawJson, "version");
        final String messagesArray = extractArray(rawJson, "messages");
        final List<String> messageObjects = extractObjectsFromArray(messagesArray);
        final List<MessageCanonical> canonicalMessages = new ArrayList<>();

        for (String messageObject : messageObjects) {
            final int id = Integer.parseInt(extractRequired(kIdPattern, messageObject, "message id"));
            final String name = extractRequired(kNamePattern, messageObject, "message name");
            final String fieldsArray = extractArray(messageObject, "fields");
            final List<String> fieldObjects = extractObjectsFromArray(fieldsArray);
            final List<FieldCanonical> fields = new ArrayList<>();
            for (String fieldObject : fieldObjects) {
                final String fieldName = extractRequired(kNamePattern, fieldObject, "field name");
                final String fieldType = extractRequired(kTypePattern, fieldObject, "field type");
                final Matcher scaleMatcher = kScalePattern.matcher(fieldObject);
                final String scale = scaleMatcher.find() ? scaleMatcher.group(1) : null;
                fields.add(new FieldCanonical(fieldName, fieldType, scale));
            }
            canonicalMessages.add(new MessageCanonical(id, name, fields));
        }

        canonicalMessages.sort(Comparator.comparingInt(MessageCanonical::id));

        final StringBuilder canonicalJson = new StringBuilder();
        canonicalJson.append("{\"messages\":[");
        for (int i = 0; i < canonicalMessages.size(); i++) {
            if (i > 0) {
                canonicalJson.append(',');
            }
            final MessageCanonical message = canonicalMessages.get(i);
            canonicalJson.append("{\"fields\":[");
            for (int j = 0; j < message.fields().size(); j++) {
                if (j > 0) {
                    canonicalJson.append(',');
                }
                final FieldCanonical field = message.fields().get(j);
                if (field.scale() == null) {
                    canonicalJson.append("{\"name\":\"")
                            .append(field.name())
                            .append("\",\"type\":\"")
                            .append(field.type())
                            .append("\"}");
                } else {
                    canonicalJson.append("{\"name\":\"")
                            .append(field.name())
                            .append("\",\"scale\":")
                            .append(field.scale())
                            .append(",\"type\":\"")
                            .append(field.type())
                            .append("\"}");
                }
            }
            canonicalJson.append("],\"id\":")
                    .append(message.id())
                    .append(",\"name\":\"")
                    .append(message.name())
                    .append("\"}");
        }
        canonicalJson.append("],\"version\":\"").append(version).append("\"}");

        return canonicalJson.toString().getBytes(StandardCharsets.UTF_8);
    }

    private static String extractRequired(Pattern pattern, String text, String label) {
        final Matcher matcher = pattern.matcher(text);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Could not parse BCNP schema " + label + ".");
        }
        return matcher.group(1);
    }

    private static String extractArray(String json, String key) {
        final int keyIndex = json.indexOf("\"" + key + "\"");
        if (keyIndex < 0) {
            throw new IllegalArgumentException("Could not parse BCNP schema array for key '" + key + "'.");
        }
        final int arrayStart = json.indexOf('[', keyIndex);
        if (arrayStart < 0) {
            throw new IllegalArgumentException("Could not locate array start for key '" + key + "'.");
        }

        int depth = 0;
        boolean inString = false;
        boolean escaping = false;
        for (int i = arrayStart; i < json.length(); i++) {
            final char ch = json.charAt(i);
            if (inString) {
                if (escaping) {
                    escaping = false;
                } else if (ch == '\\') {
                    escaping = true;
                } else if (ch == '"') {
                    inString = false;
                }
                continue;
            }
            if (ch == '"') {
                inString = true;
            } else if (ch == '[') {
                depth++;
            } else if (ch == ']') {
                depth--;
                if (depth == 0) {
                    return json.substring(arrayStart + 1, i);
                }
            }
        }
        throw new IllegalArgumentException("Could not locate array end for key '" + key + "'.");
    }

    private static List<String> extractObjectsFromArray(String arrayBody) {
        final List<String> objects = new ArrayList<>();
        int depth = 0;
        int objectStart = -1;
        boolean inString = false;
        boolean escaping = false;

        for (int i = 0; i < arrayBody.length(); i++) {
            final char ch = arrayBody.charAt(i);
            if (inString) {
                if (escaping) {
                    escaping = false;
                } else if (ch == '\\') {
                    escaping = true;
                } else if (ch == '"') {
                    inString = false;
                }
                continue;
            }

            if (ch == '"') {
                inString = true;
            } else if (ch == '{') {
                if (depth == 0) {
                    objectStart = i;
                }
                depth++;
            } else if (ch == '}') {
                depth--;
                if (depth == 0 && objectStart >= 0) {
                    objects.add(arrayBody.substring(objectStart, i + 1));
                    objectStart = -1;
                }
            }
        }
        return objects;
    }

    private record MessageCanonical(int id, String name, List<FieldCanonical> fields) {
    }

    private record FieldCanonical(String name, String type, String scale) {
    }
}
