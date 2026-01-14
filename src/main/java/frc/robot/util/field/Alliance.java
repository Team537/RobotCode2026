package frc.robot.util.field;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public enum Alliance {

    BLUE(Constants.Operator.Drive.BLUE_ALLIANCE_DRIVER_ROTATION),
    RED(Constants.Operator.Drive.RED_ALLIANCE_DRIVER_ROTATION);

    public Rotation2d driverRotation;
    Alliance(Rotation2d driverRotation) {
        this.driverRotation = driverRotation;
    }

}
