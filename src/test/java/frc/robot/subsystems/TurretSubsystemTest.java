package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.DisplayName;

import java.util.function.Supplier;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for TurretSubsystem.
 * 
 * Note: These tests focus on the logic and calculations rather than hardware interactions.
 * Hardware-dependent methods (setTurretAngle, getAngle) are tested indirectly through
 * the robot's integration tests or simulation.
 */
class TurretSubsystemTest {

    // We'll test the mathematical logic without instantiating the actual subsystem
    // since it requires hardware (TalonFX) which can't be mocked easily in unit tests.

    // ===========================
    // Field-Relative Angle Calculation Tests
    // ===========================

    @Test
    @DisplayName("Field-relative to robot-relative conversion - robot facing forward")
    void testFieldRelativeToRobotRelative_RobotForward() {
        // Robot facing 0 degrees (forward)
        Rotation2d robotHeading = Rotation2d.fromDegrees(0);
        Rotation2d fieldRelativeAngle = Rotation2d.fromDegrees(90); // Target is to the left
        
        // Expected: turret should point 90 degrees left relative to robot
        Rotation2d expectedRobotRelative = Rotation2d.fromDegrees(90);
        Rotation2d actualRobotRelative = fieldRelativeAngle.minus(robotHeading);
        
        assertEquals(expectedRobotRelative.getDegrees(), actualRobotRelative.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Field-relative to robot-relative conversion - robot rotated 45 degrees")
    void testFieldRelativeToRobotRelative_RobotRotated45() {
        // Robot facing 45 degrees
        Rotation2d robotHeading = Rotation2d.fromDegrees(45);
        Rotation2d fieldRelativeAngle = Rotation2d.fromDegrees(90); // Target is to the left on field
        
        // Expected: turret should point 45 degrees left relative to robot (90 - 45 = 45)
        Rotation2d expectedRobotRelative = Rotation2d.fromDegrees(45);
        Rotation2d actualRobotRelative = fieldRelativeAngle.minus(robotHeading);
        
        assertEquals(expectedRobotRelative.getDegrees(), actualRobotRelative.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Field-relative to robot-relative conversion - robot facing backward")
    void testFieldRelativeToRobotRelative_RobotBackward() {
        // Robot facing 180 degrees (backward)
        Rotation2d robotHeading = Rotation2d.fromDegrees(180);
        Rotation2d fieldRelativeAngle = Rotation2d.fromDegrees(0); // Target is forward on field
        
        // Expected: turret should point -180 degrees (or 180) relative to robot
        Rotation2d actualRobotRelative = fieldRelativeAngle.minus(robotHeading);
        
        // Should be -180 or 180 (equivalent)
        assertTrue(Math.abs(Math.abs(actualRobotRelative.getDegrees()) - 180) < 0.001);
    }

    @Test
    @DisplayName("Robot-relative to field-relative conversion - robot facing forward")
    void testRobotRelativeToFieldRelative_RobotForward() {
        // Robot facing 0 degrees (forward)
        Rotation2d robotHeading = Rotation2d.fromDegrees(0);
        Rotation2d robotRelativeAngle = Rotation2d.fromDegrees(45); // Turret pointing 45 degrees
        
        // Expected: field-relative should be 45 degrees
        Rotation2d expectedFieldRelative = Rotation2d.fromDegrees(45);
        Rotation2d actualFieldRelative = robotRelativeAngle.plus(robotHeading);
        
        assertEquals(expectedFieldRelative.getDegrees(), actualFieldRelative.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Robot-relative to field-relative conversion - robot rotated 90 degrees")
    void testRobotRelativeToFieldRelative_RobotRotated90() {
        // Robot facing 90 degrees (left)
        Rotation2d robotHeading = Rotation2d.fromDegrees(90);
        Rotation2d robotRelativeAngle = Rotation2d.fromDegrees(45); // Turret pointing 45 degrees relative
        
        // Expected: field-relative should be 135 degrees (90 + 45)
        Rotation2d expectedFieldRelative = Rotation2d.fromDegrees(135);
        Rotation2d actualFieldRelative = robotRelativeAngle.plus(robotHeading);
        
        assertEquals(expectedFieldRelative.getDegrees(), actualFieldRelative.getDegrees(), 0.001);
    }

    // ===========================
    // Target Calculation Tests
    // ===========================

    @Test
    @DisplayName("Calculate angle to target - target directly forward")
    void testCalculateAngleToTarget_DirectlyForward() {
        Translation2d robotPosition = new Translation2d(0, 0);
        Translation2d targetPosition = new Translation2d(5, 0); // 5 meters forward
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        assertEquals(0.0, angle.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Calculate angle to target - target to the left")
    void testCalculateAngleToTarget_Left() {
        Translation2d robotPosition = new Translation2d(0, 0);
        Translation2d targetPosition = new Translation2d(0, 5); // 5 meters to the left
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        assertEquals(90.0, angle.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Calculate angle to target - target at 45 degrees")
    void testCalculateAngleToTarget_45Degrees() {
        Translation2d robotPosition = new Translation2d(0, 0);
        Translation2d targetPosition = new Translation2d(5, 5); // 45 degrees
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        assertEquals(45.0, angle.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Calculate angle to target - target behind")
    void testCalculateAngleToTarget_Behind() {
        Translation2d robotPosition = new Translation2d(0, 0);
        Translation2d targetPosition = new Translation2d(-5, 0); // Behind
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        // Should be 180 or -180
        assertTrue(Math.abs(Math.abs(angle.getDegrees()) - 180) < 0.001);
    }

    @Test
    @DisplayName("Calculate angle to target - robot not at origin")
    void testCalculateAngleToTarget_RobotNotAtOrigin() {
        Translation2d robotPosition = new Translation2d(3, 2);
        Translation2d targetPosition = new Translation2d(8, 2); // 5 meters forward from robot
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        assertEquals(0.0, angle.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Calculate angle to target - diagonal movement")
    void testCalculateAngleToTarget_Diagonal() {
        Translation2d robotPosition = new Translation2d(1, 1);
        Translation2d targetPosition = new Translation2d(4, 5); // 3 right, 4 up
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        // atan2(4, 3) = ~53.13 degrees
        assertEquals(53.13, angle.getDegrees(), 0.1);
    }

    // ===========================
    // Translation3d to Translation2d Conversion Tests
    // ===========================

    @Test
    @DisplayName("Translation3d to Translation2d conversion - ground level")
    void testTranslation3dTo2d_GroundLevel() {
        Translation3d translation3d = new Translation3d(5, 3, 0);
        Translation2d translation2d = translation3d.toTranslation2d();
        
        assertEquals(5.0, translation2d.getX(), 0.001);
        assertEquals(3.0, translation2d.getY(), 0.001);
    }

    @Test
    @DisplayName("Translation3d to Translation2d conversion - elevated target")
    void testTranslation3dTo2d_Elevated() {
        Translation3d translation3d = new Translation3d(5, 3, 2); // 2 meters high
        Translation2d translation2d = translation3d.toTranslation2d();
        
        // Z coordinate should be ignored for horizontal aiming
        assertEquals(5.0, translation2d.getX(), 0.001);
        assertEquals(3.0, translation2d.getY(), 0.001);
    }

    // ===========================
    // Pose Supplier Tests
    // ===========================

    @Test
    @DisplayName("Default pose supplier returns zero pose")
    void testDefaultPoseSupplier() {
        Supplier<Pose2d> defaultSupplier = () -> Pose2d.kZero;
        Pose2d pose = defaultSupplier.get();
        
        assertEquals(0.0, pose.getX(), 0.001);
        assertEquals(0.0, pose.getY(), 0.001);
        assertEquals(0.0, pose.getRotation().getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Custom pose supplier returns correct pose")
    void testCustomPoseSupplier() {
        Pose2d customPose = new Pose2d(5, 3, Rotation2d.fromDegrees(45));
        Supplier<Pose2d> customSupplier = () -> customPose;
        Pose2d pose = customSupplier.get();
        
        assertEquals(5.0, pose.getX(), 0.001);
        assertEquals(3.0, pose.getY(), 0.001);
        assertEquals(45.0, pose.getRotation().getDegrees(), 0.001);
    }

    // ===========================
    // Integration/Scenario Tests
    // ===========================

    @Test
    @DisplayName("Full scenario - robot at (2,2) facing 90°, target at (4,6)")
    void testFullScenario_RobotRotatedTargetOffset() {
        // Setup
        Pose2d robotPose = new Pose2d(2, 2, Rotation2d.fromDegrees(90));
        Translation3d targetTranslation3d = new Translation3d(4, 6, 0);
        
        // Calculate displacement
        Translation2d targetTranslation = targetTranslation3d.toTranslation2d();
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d displacement = targetTranslation.minus(robotTranslation);
        
        // Calculate field-relative angle to target
        Rotation2d fieldRelativeAngle = new Rotation2d(displacement.getX(), displacement.getY());
        
        // Convert to robot-relative
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotHeading);
        
        // Verify calculations
        // Displacement: (4-2, 6-2) = (2, 4)
        assertEquals(2.0, displacement.getX(), 0.001);
        assertEquals(4.0, displacement.getY(), 0.001);
        
        // Field-relative angle: atan2(4, 2) ≈ 63.43 degrees
        assertEquals(63.43, fieldRelativeAngle.getDegrees(), 0.1);
        
        // Robot-relative: 63.43 - 90 = -26.57 degrees
        assertEquals(-26.57, robotRelativeAngle.getDegrees(), 0.1);
    }

    @Test
    @DisplayName("Full scenario - robot at origin facing forward, target behind and left")
    void testFullScenario_TargetBehindLeft() {
        // Setup
        Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        Translation3d targetTranslation3d = new Translation3d(-3, 4, 0); // Behind and left
        
        // Calculate displacement
        Translation2d targetTranslation = targetTranslation3d.toTranslation2d();
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d displacement = targetTranslation.minus(robotTranslation);
        
        // Calculate field-relative angle to target
        Rotation2d fieldRelativeAngle = new Rotation2d(displacement.getX(), displacement.getY());
        
        // Convert to robot-relative
        Rotation2d robotHeading = robotPose.getRotation();
        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotHeading);
        
        // Displacement: (-3, 4)
        assertEquals(-3.0, displacement.getX(), 0.001);
        assertEquals(4.0, displacement.getY(), 0.001);
        
        // Field-relative angle: atan2(4, -3) ≈ 126.87 degrees
        assertEquals(126.87, fieldRelativeAngle.getDegrees(), 0.1);
        
        // Robot-relative: same as field-relative since robot at 0 degrees
        assertEquals(126.87, robotRelativeAngle.getDegrees(), 0.1);
    }

    // ===========================
    // Edge Case Tests
    // ===========================

    @Test
    @DisplayName("Target at same position as robot")
    void testTargetAtSamePosition() {
        Translation2d robotPosition = new Translation2d(5, 5);
        Translation2d targetPosition = new Translation2d(5, 5);
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        
        // Displacement should be zero
        assertEquals(0.0, displacement.getX(), 0.001);
        assertEquals(0.0, displacement.getY(), 0.001);
        
        // Angle is undefined when displacement is zero, but Rotation2d handles it
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        assertEquals(0.0, angle.getDegrees(), 0.001);
    }

    @Test
    @DisplayName("Very small displacement")
    void testVerySmallDisplacement() {
        Translation2d robotPosition = new Translation2d(0, 0);
        Translation2d targetPosition = new Translation2d(0.001, 0.001);
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        // Should be 45 degrees
        assertEquals(45.0, angle.getDegrees(), 0.1);
    }

    @Test
    @DisplayName("Very large displacement")
    void testVeryLargeDisplacement() {
        Translation2d robotPosition = new Translation2d(0, 0);
        Translation2d targetPosition = new Translation2d(1000, 1000);
        
        Translation2d displacement = targetPosition.minus(robotPosition);
        Rotation2d angle = new Rotation2d(displacement.getX(), displacement.getY());
        
        // Should still be 45 degrees
        assertEquals(45.0, angle.getDegrees(), 0.1);
    }

    @Test
    @DisplayName("Angle wraparound - 360 degrees")
    void testAngleWraparound() {
        Rotation2d angle2 = Rotation2d.fromDegrees(370); // Wraps to 10 degrees
        
        // The degree values should be equivalent when normalized
        // 370 degrees normalized should be 10 degrees
        double normalizedAngle2 = angle2.getDegrees();
        if (normalizedAngle2 > 180) {
            normalizedAngle2 -= 360;
        }
        
        assertEquals(10.0, normalizedAngle2, 0.001);
    }

    @Test
    @DisplayName("Negative angle handling")
    void testNegativeAngle() {
        Rotation2d robotHeading = Rotation2d.fromDegrees(30);
        Rotation2d fieldRelativeAngle = Rotation2d.fromDegrees(10);
        
        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotHeading);
        
        // Should be -20 degrees
        assertEquals(-20.0, robotRelativeAngle.getDegrees(), 0.001);
    }
}
