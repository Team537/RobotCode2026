package frc.robot.util.turret;

import frc.robot.Constants;

public enum TargetingStrategy {
    SHOOTING(Constants.Turret.SHOOTING_SOLVER_CONFIG),
    PASSING(Constants.Turret.PASSING_SOLVER_CONFIG);

    private TurretSolver.Config config;

    TargetingStrategy(TurretSolver.Config config) {
        this.config = config;
    };

    public TurretSolver.Config getConfig() {
        return config;
    }
}
