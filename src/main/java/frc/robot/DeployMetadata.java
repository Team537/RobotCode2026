package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class DeployMetadata {
    private DeployMetadata() {}

    public static void publishAtStartup() {
        publishFromFile(Configs.DEPLOY_METADATA_PATH);
    }

    public static void publishFromFile(String absolutePath) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(Configs.NT_TABLE);

        String status;
        String raw = "";
        Map<String, String> parsed = new LinkedHashMap<>();

        try {
            Path p = Path.of(absolutePath);
            if (!Files.exists(p)) {
                status = "missing: " + absolutePath;
            } else {
                raw = Files.readString(p, StandardCharsets.UTF_8);
                parsed = parseKeyValueLines(Files.readAllLines(p, StandardCharsets.UTF_8));
                status = "ok";
            }
        } catch (Exception e) {
            status = "error: " + e.getClass().getSimpleName();
        }

        publishToNetworkTable(table, status, raw, parsed);
    }

    /**
     * Publishes metadata to a NetworkTable. This method is separated out to allow
     * for easier testing by accepting pre-parsed data and a NetworkTable instance.
     *
     * @param table The NetworkTable to publish to
     * @param status The status message ("ok", "missing: ...", "error: ...")
     * @param raw The raw file contents
     * @param parsed The parsed key-value pairs
     */
    public static void publishToNetworkTable(NetworkTable table, String status, String raw, Map<String, String> parsed) {
        // Publish status and raw contents for Elastic/Shuffleboard widgets.
        table.getEntry(Configs.NT_STATUS_KEY).setString(status);
        table.getEntry(Configs.NT_RAW_KEY).setString(raw);

        // Publish parsed key-value lines under a prefix for targeted widgets.
        for (Map.Entry<String, String> entry : parsed.entrySet()) {
            String key = Configs.NT_PREFIX_PARSED + sanitizeKey(entry.getKey());
            table.getEntry(key).setString(entry.getValue());
        }
    }

    /**
     * Publishes metadata from pre-read lines. Useful for testing without file system dependencies.
     *
     * @param table The NetworkTable to publish to
     * @param lines The lines to parse and publish
     */
    public static void publishFromLines(NetworkTable table, List<String> lines) {
        Map<String, String> parsed = parseKeyValueLines(lines);
        String raw = String.join("\n", lines);
        publishToNetworkTable(table, "ok", raw, parsed);
    }

    /**
     * Parses lines of the form "key=value" or "key: value" into a map.
     * Blank lines and comment lines starting with '#' or ';' are ignored.
     * Package-private for testing.
     */
    static Map<String, String> parseKeyValueLines(List<String> lines) {
        Map<String, String> out = new LinkedHashMap<>();
        for (String line : lines) {
            if (line == null) continue;
            String trimmed = line.trim();
            if (trimmed.isEmpty()) continue;
            if (trimmed.startsWith("#") || trimmed.startsWith(";")) continue;

            int eq = trimmed.indexOf('=');
            int colon = trimmed.indexOf(':');

            int splitIdx;
            if (eq >= 0 && colon >= 0) {
                splitIdx = Math.min(eq, colon);
            } else if (eq >= 0) {
                splitIdx = eq;
            } else if (colon >= 0) {
                splitIdx = colon;
            } else {
                // no delimiter
                continue;
            }

            String key = trimmed.substring(0, splitIdx).trim();
            String val = trimmed.substring(splitIdx + 1).trim();
            if (!key.isEmpty()) {
                out.put(key, val);
            }
        }
        return out;
    }

    /**
     * Sanitizes a key for use in NetworkTables topic names.
     * Package-private for testing.
     */
    static String sanitizeKey(String key) {
        // NT topic names should avoid spaces or special characters.
        return key.replaceAll("\\W+", "_");
    }
}
