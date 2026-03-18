package frc.robot.util.debug;

public enum TextColor {
    
    RESET("m"),
    BLACK("30m"),
    RED("31m"),
    GREEN("32m"),
    YELLOW("33m"),
    BLUE("34m"),
    MAGENTA("35m"),
    CYAN("36m"),
    WHITE("37m"  );

    /**
     * Constructs a {@code TextColor} enum with the respective escape string. 
     */
    private final String ESCAPE_STRING;
    private TextColor(String escapeString) {
        this.ESCAPE_STRING = escapeString;
    }

    /**
     * Returns this {@code TextColor}'s respective escape string.'
     * 
     * @return This {@code TextColor}'s respective escape string.'
     */
    public String getEscapeString() {
        return this.ESCAPE_STRING;
    }
}
