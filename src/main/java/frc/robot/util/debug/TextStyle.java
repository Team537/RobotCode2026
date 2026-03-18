package frc.robot.util.debug;

public enum TextStyle {
    
    BOLD("1;"),
    UNDERLINE("4;"),
    REVERSE("7;");

    /**
     * Constructs a {@code TextStyle} enum with the respective escape string. 
     */
    private final String FORMAT_STRING;
    private TextStyle(String formatString) { 
        this.FORMAT_STRING = formatString;
    }

    /**
     * Returns this {@code TextStyle}'s respective escape string.'
     * 
     * @return This {@code TextStyle}'s respective escape string.'
     */
    public String getFormatCode() {
        return this.FORMAT_STRING;
    }
}
