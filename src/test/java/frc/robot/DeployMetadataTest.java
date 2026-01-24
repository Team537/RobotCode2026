package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

/**
 * Unit tests for DeployMetadata class.
 * 
 * These tests focus on the parsing logic and NetworkTable publishing without
 * requiring actual file system access or NetworkTables initialization.
 */
class DeployMetadataTest {

    private NetworkTable mockTable;
    private NetworkTableEntry mockEntry;

    @BeforeEach
    void setUp() {
        mockTable = mock(NetworkTable.class);
        mockEntry = mock(NetworkTableEntry.class);
        
        // Default behavior: return mock entry for any getEntry call
        when(mockTable.getEntry(anyString())).thenReturn(mockEntry);
        when(mockEntry.setString(anyString())).thenReturn(true);
    }

    // ===========================
    // parseKeyValueLines() Tests
    // ===========================

    @Test
    @DisplayName("parseKeyValueLines - parses equals sign format")
    void testParseKeyValueLines_EqualsFormat() {
        List<String> lines = Arrays.asList(
            "user=jason",
            "host=desktop-123",
            "gitBranch=main"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(3, result.size());
        assertEquals("jason", result.get("user"));
        assertEquals("desktop-123", result.get("host"));
        assertEquals("main", result.get("gitBranch"));
    }

    @Test
    @DisplayName("parseKeyValueLines - parses colon format")
    void testParseKeyValueLines_ColonFormat() {
        List<String> lines = Arrays.asList(
            "user: jason",
            "host: desktop-123",
            "gitBranch: feature/test"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(3, result.size());
        assertEquals("jason", result.get("user"));
        assertEquals("desktop-123", result.get("host"));
        assertEquals("feature/test", result.get("gitBranch"));
    }

    @Test
    @DisplayName("parseKeyValueLines - handles mixed formats")
    void testParseKeyValueLines_MixedFormats() {
        List<String> lines = Arrays.asList(
            "user=jason",
            "host: desktop-123",
            "gitBranch=main"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(3, result.size());
        assertEquals("jason", result.get("user"));
        assertEquals("desktop-123", result.get("host"));
    }

    @Test
    @DisplayName("parseKeyValueLines - ignores blank lines")
    void testParseKeyValueLines_IgnoresBlankLines() {
        List<String> lines = Arrays.asList(
            "user=jason",
            "",
            "   ",
            "host=desktop-123"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertEquals("jason", result.get("user"));
        assertEquals("desktop-123", result.get("host"));
    }

    @Test
    @DisplayName("parseKeyValueLines - ignores comment lines starting with #")
    void testParseKeyValueLines_IgnoresHashComments() {
        List<String> lines = Arrays.asList(
            "# This is a comment",
            "user=jason",
            "#another comment",
            "host=desktop-123"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertEquals("jason", result.get("user"));
    }

    @Test
    @DisplayName("parseKeyValueLines - ignores comment lines starting with ;")
    void testParseKeyValueLines_IgnoresSemicolonComments() {
        List<String> lines = Arrays.asList(
            "; This is a comment",
            "user=jason",
            ";another comment",
            "host=desktop-123"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertEquals("jason", result.get("user"));
    }

    @Test
    @DisplayName("parseKeyValueLines - ignores lines without delimiters")
    void testParseKeyValueLines_IgnoresLinesWithoutDelimiters() {
        List<String> lines = Arrays.asList(
            "user=jason",
            "this line has no delimiter",
            "host=desktop-123"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertFalse(result.containsKey("this line has no delimiter"));
    }

    @Test
    @DisplayName("parseKeyValueLines - trims whitespace from keys and values")
    void testParseKeyValueLines_TrimsWhitespace() {
        List<String> lines = Arrays.asList(
            "  user  =  jason  ",
            "host:   desktop-123   "
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertEquals("jason", result.get("user"));
        assertEquals("desktop-123", result.get("host"));
    }

    @Test
    @DisplayName("parseKeyValueLines - handles values containing delimiters")
    void testParseKeyValueLines_ValuesWithDelimiters() {
        List<String> lines = Arrays.asList(
            "timestamp=2026-01-21T10:30:00-05:00",
            "url=https://github.com/Team537/RobotCode2026"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertEquals("2026-01-21T10:30:00-05:00", result.get("timestamp"));
        assertEquals("https://github.com/Team537/RobotCode2026", result.get("url"));
    }

    @Test
    @DisplayName("parseKeyValueLines - uses first delimiter when multiple present")
    void testParseKeyValueLines_MultipleDelimiters() {
        List<String> lines = Arrays.asList(
            "key1=value:with:colons",
            "key2:value=with=equals"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertEquals("value:with:colons", result.get("key1"));
        assertEquals("value=with=equals", result.get("key2"));
    }

    @Test
    @DisplayName("parseKeyValueLines - handles empty values")
    void testParseKeyValueLines_EmptyValues() {
        List<String> lines = Arrays.asList(
            "user=",
            "host=desktop-123"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(2, result.size());
        assertEquals("", result.get("user"));
        assertEquals("desktop-123", result.get("host"));
    }

    @Test
    @DisplayName("parseKeyValueLines - handles empty list")
    void testParseKeyValueLines_EmptyList() {
        List<String> lines = Arrays.asList();

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertTrue(result.isEmpty());
    }

    @Test
    @DisplayName("parseKeyValueLines - preserves insertion order")
    void testParseKeyValueLines_PreservesOrder() {
        List<String> lines = Arrays.asList(
            "user=jason",
            "host=desktop-123",
            "gitBranch=main",
            "gitCommit=abc123",
            "timestamp=2026-01-21T10:30:00"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        // LinkedHashMap should preserve order
        assertInstanceOf(LinkedHashMap.class, result);
        List<String> keys = result.keySet().stream().toList();
        assertEquals("user", keys.get(0));
        assertEquals("host", keys.get(1));
        assertEquals("gitBranch", keys.get(2));
        assertEquals("gitCommit", keys.get(3));
        assertEquals("timestamp", keys.get(4));
    }

    // ===========================
    // sanitizeKey() Tests
    // ===========================

    @Test
    @DisplayName("sanitizeKey - replaces spaces with underscores")
    void testSanitizeKey_Spaces() {
        String result = DeployMetadata.sanitizeKey("git branch");
        assertEquals("git_branch", result);
    }

    @Test
    @DisplayName("sanitizeKey - replaces special characters")
    void testSanitizeKey_SpecialCharacters() {
        String result = DeployMetadata.sanitizeKey("key@name#test");
        assertEquals("key_name_test", result);
    }

    @Test
    @DisplayName("sanitizeKey - keeps alphanumeric and underscores")
    void testSanitizeKey_AlphanumericAndUnderscores() {
        String result = DeployMetadata.sanitizeKey("valid_key_123");
        assertEquals("valid_key_123", result);
    }

    @Test
    @DisplayName("sanitizeKey - handles consecutive special characters")
    void testSanitizeKey_ConsecutiveSpecialChars() {
        String result = DeployMetadata.sanitizeKey("key!!!name");
        assertEquals("key_name", result);
    }

    @Test
    @DisplayName("sanitizeKey - handles empty string")
    void testSanitizeKey_EmptyString() {
        String result = DeployMetadata.sanitizeKey("");
        assertEquals("", result);
    }

    // ===========================
    // publishToNetworkTable() Tests
    // ===========================

    @Test
    @DisplayName("publishToNetworkTable - publishes status")
    void testPublishToNetworkTable_PublishesStatus() {
        Map<String, String> parsed = new LinkedHashMap<>();
        
        DeployMetadata.publishToNetworkTable(mockTable, "ok", "", parsed);

        verify(mockTable).getEntry(Configs.NT_STATUS_KEY);
        verify(mockEntry).setString("ok");
    }

    @Test
    @DisplayName("publishToNetworkTable - publishes raw content")
    void testPublishToNetworkTable_PublishesRaw() {
        Map<String, String> parsed = new LinkedHashMap<>();
        String rawContent = "user=jason\nhost=desktop";
        
        DeployMetadata.publishToNetworkTable(mockTable, "ok", rawContent, parsed);

        verify(mockTable).getEntry(Configs.NT_RAW_KEY);
        verify(mockEntry).setString(rawContent);
    }

    @Test
    @DisplayName("publishToNetworkTable - publishes parsed entries")
    void testPublishToNetworkTable_PublishesParsedEntries() {
        Map<String, String> parsed = new LinkedHashMap<>();
        parsed.put("user", "jason");
        parsed.put("host", "desktop-123");
        
        DeployMetadata.publishToNetworkTable(mockTable, "ok", "", parsed);

        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "user");
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "host");
        verify(mockEntry, atLeastOnce()).setString("jason");
        verify(mockEntry, atLeastOnce()).setString("desktop-123");
    }

    @Test
    @DisplayName("publishToNetworkTable - sanitizes keys before publishing")
    void testPublishToNetworkTable_SanitizesKeys() {
        Map<String, String> parsed = new LinkedHashMap<>();
        parsed.put("git branch", "main");
        
        DeployMetadata.publishToNetworkTable(mockTable, "ok", "", parsed);

        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "git_branch");
        verify(mockEntry).setString("main");
    }

    @Test
    @DisplayName("publishToNetworkTable - handles empty parsed map")
    void testPublishToNetworkTable_EmptyParsedMap() {
        Map<String, String> parsed = new LinkedHashMap<>();
        
        DeployMetadata.publishToNetworkTable(mockTable, "ok", "", parsed);

        // Should still publish status and raw
        verify(mockTable).getEntry(Configs.NT_STATUS_KEY);
        verify(mockTable).getEntry(Configs.NT_RAW_KEY);
        // But no parsed entries
        verify(mockTable, never()).getEntry(startsWith(Configs.NT_PREFIX_PARSED));
    }

    // ===========================
    // publishFromLines() Tests
    // ===========================

    @Test
    @DisplayName("publishFromLines - parses and publishes lines")
    void testPublishFromLines() {
        List<String> lines = Arrays.asList(
            "user=jason",
            "host=desktop-123",
            "gitBranch=main"
        );

        DeployMetadata.publishFromLines(mockTable, lines);

        // Verify status is "ok"
        verify(mockEntry).setString("ok");
        
        // Verify raw is the joined lines
        verify(mockEntry).setString("user=jason\nhost=desktop-123\ngitBranch=main");
        
        // Verify parsed entries
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "user");
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "host");
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "gitBranch");
    }

    @Test
    @DisplayName("publishFromLines - handles empty lines list")
    void testPublishFromLines_EmptyList() {
        List<String> lines = Arrays.asList();

        DeployMetadata.publishFromLines(mockTable, lines);

        verify(mockEntry).setString("ok");
        verify(mockEntry).setString(""); // empty raw content
    }

    // ===========================
    // Integration-style Tests
    // ===========================

    @Test
    @DisplayName("Full workflow - parse real metadata format")
    void testFullWorkflow_RealMetadataFormat() {
        List<String> lines = Arrays.asList(
            "user=jason",
            "host=DESKTOP-ABC123",
            "gitBranch=feature/track-deployment",
            "gitCommit=a1b2c3d4e5f6g7h8i9j0",
            "timestamp=2026-01-21T10:30:45.123-05:00"
        );

        Map<String, String> result = DeployMetadata.parseKeyValueLines(lines);

        assertEquals(5, result.size());
        assertEquals("jason", result.get("user"));
        assertEquals("DESKTOP-ABC123", result.get("host"));
        assertEquals("feature/track-deployment", result.get("gitBranch"));
        assertEquals("a1b2c3d4e5f6g7h8i9j0", result.get("gitCommit"));
        assertEquals("2026-01-21T10:30:45.123-05:00", result.get("timestamp"));

        // Now publish it
        DeployMetadata.publishFromLines(mockTable, lines);

        // Verify all keys were published with sanitized names
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "user");
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "host");
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "gitBranch");
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "gitCommit");
        verify(mockTable).getEntry(Configs.NT_PREFIX_PARSED + "timestamp");
    }
}
