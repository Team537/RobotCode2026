package frc.robot.util.debug;

public enum TextBackgroundColor {
    
     BLACK_BACKGROUND("40m"),
     RED_BACKGROUND("41m"),
     GREEN_BACKGROUND("42m"),
     YELLOW_BACKGROUND("43m"),
     BLUE_BACKGROUND("44m"),
     MAGENTA_BACKGROUND("45m"),
     CYAN_BACKGROUND("46m"),
     WHITE_BACKGROUND("47m");

     /**
      * Constructs a TextBackground enum value and ties it to the correlated escape string.
      */
    private final String ESCAPE_STRING;
    private TextBackgroundColor(String escapeString) {
        this.ESCAPE_STRING = escapeString;
    }

    /**
     * Returns this {@code TextBackgroundColor}'s respective escape string.'
     * 
     * @return This {@code TextBackgroundColor}'s respective escape string.'
     */
    public String getEscapeString() {
        return this.ESCAPE_STRING;
    }
}
