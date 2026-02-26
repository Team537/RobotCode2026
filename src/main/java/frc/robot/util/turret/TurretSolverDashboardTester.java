package frc.robot.util.turret;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSolverDashboardTester {

    private final TurretSolver.Config config;

    public TurretSolverDashboardTester(TurretSolver.Config config) {
        this.config = config;

        // === Default input values published once ===
        SmartDashboard.putNumber("Solver/Robot X", 0.0);
        SmartDashboard.putNumber("Solver/Robot Y", 0.0);
        SmartDashboard.putNumber("Solver/Robot Yaw Deg", 0.0);

        SmartDashboard.putNumber("Solver/Robot Vx", 0.0);
        SmartDashboard.putNumber("Solver/Robot Vy", 0.0);

        SmartDashboard.putNumber("Solver/Target X", 5.0);
        SmartDashboard.putNumber("Solver/Target Y", 0.0);
        SmartDashboard.putNumber("Solver/Target Z", 2.0);
    }

    public void periodic() {

        // ================= INPUTS =================

        double robotX = SmartDashboard.getNumber("Solver/Robot X", 0);
        double robotY = SmartDashboard.getNumber("Solver/Robot Y", 0);
        double robotYawDeg = SmartDashboard.getNumber("Solver/Robot Yaw Deg", 0);

        double vx = SmartDashboard.getNumber("Solver/Robot Vx", 0);
        double vy = SmartDashboard.getNumber("Solver/Robot Vy", 0);

        double targetX = SmartDashboard.getNumber("Solver/Target X", 0);
        double targetY = SmartDashboard.getNumber("Solver/Target Y", 0);
        double targetZ = SmartDashboard.getNumber("Solver/Target Z", 0);

        Pose2d robotPose = new Pose2d(
                robotX,
                robotY,
                Rotation2d.fromDegrees(robotYawDeg)
        );

        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, 0);

        Translation3d target = new Translation3d(targetX, targetY, targetZ);

        // ================= SOLVE =================

        TurretSolver.State result =
                TurretSolver.solve(robotPose, speeds, target, config);

        // ================= OUTPUTS =================

        SmartDashboard.putBoolean("Solver/Valid", result.isValid());
        SmartDashboard.putNumber("Solver/Launch Velocity", result.getLaunchVelocity());
        SmartDashboard.putNumber("Solver/Yaw Deg", result.getYaw().getDegrees());
        SmartDashboard.putNumber("Solver/Pitch Deg", result.getPitch().getDegrees());
        SmartDashboard.putNumber("Solver/Max Height", result.getMaxHeight());
        SmartDashboard.putNumber("Solver/Impact Velocity", result.getImpactVelocity());
    }
}