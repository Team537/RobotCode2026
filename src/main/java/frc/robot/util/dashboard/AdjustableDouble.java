package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AdjustableDouble extends AdjustableValue<Double> {

    private Integer roundingPlaces = null; // null = no rounding

    public AdjustableDouble(String key, double initial, double min, double max) {
        super(key, initial, min, max);
    }

    /**
     * Sets how many decimal places to round to when pushing to dashboard.
     * @param places the number of decimal places to round to
     * 
     * Examples:
     * 2 - round to 2 decimal places (0.01)
     * 1 - round to 1 decimal place (0.1)
     * 0 - round to whole number
     * -1 - round to nearest 10
     * -2 - round to nearest 100
     */
    public void setDashboardRounding(int places) {
        this.roundingPlaces = places;
        pushToDashboard(); // refresh immediately
    }

    @Override
    protected Double clamp(Double val) {
        return Math.min(max, Math.max(min, val));
    }

    @Override
    protected Double addInternal(Double a, Double b) {
        return a + b;
    }

    @Override
    protected Double readFromDashboard() {
        return SmartDashboard.getNumber(key, value);
    }

    @Override
    protected void pushToDashboard() {
        double output = value;

        if (roundingPlaces != null) {
            double scale = Math.pow(10, roundingPlaces);
            output = Math.round(value * scale) / scale;
        }

        SmartDashboard.putNumber(key, output);
    }
}
