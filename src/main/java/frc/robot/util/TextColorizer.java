package frc.robot.util;

import frc.robot.util.debug.TextBackgroundColor;
import frc.robot.util.debug.TextColor;
import frc.robot.util.debug.TextStyle;

public class TextColorizer {
    
    // Universal Constants
    public static final String RESET = "\u001b[0m";
    public static final String BASE_ESCAPE_SEQUENCE = "\u001b[%s%s";

    /**
     * Formats the text such that it will be displayed with the requested color in an ANSI escape sequence supported terminal.
     * 
     * @param text The text to format
     * @param textColor The color that the output text will be displayed as.
     * @return A new string with the necessary escape sequences to display the desired color of text
     */
    public static String colorizeText(String text, TextColor textColor) {
        return String.format(BASE_ESCAPE_SEQUENCE, "", textColor.getEscapeString()) + text + RESET;
    }

    /**
     * Formats the text to the given text style. This will only work in ANSI escape sequence supported terminals.
     * 
     * @param text The text to be formatted.
     * @param textStyle The style to format the text to.
     * @return A string with the necessary escape characters to apply the desired format.
     */
    public String stylizeText(String text, TextStyle textStyle) {
        return String.format(BASE_ESCAPE_SEQUENCE, textStyle.getFormatCode(), TextColor.RESET) + text + RESET;
    }

    /**
     * Formats the given text to the desired style and color. This will only work in ANSI escape sequence supported terminals.
     * 
     * @param text The text to be formatted.
     * @param textStyle The style to format the text to.
     * @param textColor The color the text will be displayed as.
     * @return A string with the necessary escape characters to apply the desired format and color.
     */
    public static String formatText(String text, TextStyle textStyle, TextColor textColor) {
        return String.format(BASE_ESCAPE_SEQUENCE, textStyle.getFormatCode() + ";" ,textColor.getEscapeString()) + text + RESET;
    }

    /**
     * Formats the text to have a desired background color. This will only work in ANSI escape sequence supported terminals.
     * 
     * @param text The text to be formatted.
     * @param backgroundColor The background color to be applied to the text.
     * @return A string with the necessary escape characters to apply the desired background color.
     */
    public static String colorizeTextBackground(String text, TextBackgroundColor backgroundColor) {
        return String.format(BASE_ESCAPE_SEQUENCE, "", backgroundColor.getEscapeString()) + text + RESET;
    }
}
