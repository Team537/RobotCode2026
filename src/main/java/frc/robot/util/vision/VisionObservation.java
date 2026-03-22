package frc.robot.util.vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionObservation {
  public final Cameras camera;
  public final EstimatedRobotPose estimate;
  public final Pose2d pose;
  public final double timestamp;
  public final int tagCount;
  public final double avgTagDistance;
  public final double ambiguity;
  public final boolean isMultiTag;
  public final Matrix<N3, N1> stdDevs;

  public VisionObservation(
      Cameras camera,
      EstimatedRobotPose estimate,
      Pose2d pose,
      double timestamp,
      int tagCount,
      double avgTagDistance,
      double ambiguity,
      boolean isMultiTag,
      Matrix<N3, N1> stdDevs) {
    this.camera = camera;
    this.estimate = estimate;
    this.pose = pose;
    this.timestamp = timestamp;
    this.tagCount = tagCount;
    this.avgTagDistance = avgTagDistance;
    this.ambiguity = ambiguity;
    this.isMultiTag = isMultiTag;
    this.stdDevs = stdDevs;
  }
}