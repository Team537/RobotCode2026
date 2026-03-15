package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.vision.Cameras;
import frc.robot.util.vision.VisionObservation;

import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.databind.ser.impl.PropertySerializerMap;

import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Example PhotonVision class to aid in the pursuit of accurate odometry. Taken
 * from
 * https://gitlab.com/ironclad_code/ironclad-2024/-/blob/master/src/main/java/frc/robot/vision/Vision.java?ref_type=heads
 */
public class PhotonVisionOdometry {

  /**
   * April Tag Field Layout of the year.
   */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2026RebuiltAndymark);

  /**
   * Ambiguity defined as a value between (0,1). Used in
   * {@link PhotonVisionOdometry#filterPose}.
   */
  private final double maximumAmbiguity = 0.25;
  /**
   * Photon Vision Simulation
   */
  public VisionSystemSim visionSim;
  /**
   * Count of times that the odom thinks we're more than 10meters away from the
   * april tag.
   */
  private double longDistangePoseEstimationCount = 0;
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier, should reference
   *                    {@link SwerveDrive#getPose()}
   * @param field       Current field, should be {@link SwerveDrive#field}
   */
  public PhotonVisionOdometry(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim("Vision");
      visionSim.addAprilTags(fieldLayout);

      for (Cameras c : Cameras.values()) {
        c.addToVisionSim(visionSim);
      }

      openSimCameraViews();
    }
  }

  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to
   *                    the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }
  
  /*
   * POSITION ESTIMATION
   */
  private double averageTagDistance(EstimatedRobotPose est) {
    if (est.targetsUsed == null || est.targetsUsed.isEmpty()) return Double.POSITIVE_INFINITY;

    double sum = 0.0;
    for (var target : est.targetsUsed) {
      sum += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    return sum / est.targetsUsed.size();
  }

  private double maxAmbiguity(EstimatedRobotPose est) {
    if (est.targetsUsed == null || est.targetsUsed.isEmpty()) return 1.0;

    double max = 0.0;
    for (var target : est.targetsUsed) {
      max = Math.max(max, target.getPoseAmbiguity());
    }
    return max;
  }

  private boolean agrees(Pose2d posA, Pose2d posB, double maxXY, double maxThetaRad) {
    return posA.getTranslation().getDistance(posB.getTranslation()) < maxXY
        && Math.abs(posA.getRotation().minus(posB.getRotation()).getRadians()) < maxThetaRad;
  }

  private List<VisionObservation> filterByConsensus(List<VisionObservation> obs) {
    List<VisionObservation> accepted = new ArrayList<>();

    for (VisionObservation obsA : obs) {
      int agreements = 0;
      for (VisionObservation obsB : obs) {
        if (obsA == obsB) continue;
        if (agrees(obsA.pose, obsB.pose, 0.6, Math.toRadians(15))) {
          agreements++;
        }
      }

      // keep if another camera agrees, or if it is strong multitag
      if (agreements > 0 || (obsA.isMultiTag && obsA.tagCount >= 2 && obsA.avgTagDistance < 4.0)) {
        accepted.add(obsA);
      }
    }

    return accepted;
  }

  private boolean shouldAcceptEnabledMeasurement(
      VisionObservation obs,
      Pose2d currentPose,
      ChassisSpeeds speeds,
      int supportCount) {

    double translationalError =
        currentPose.getTranslation().getDistance(obs.pose.getTranslation());
    double headingError =
        Math.abs(currentPose.getRotation().minus(obs.pose.getRotation()).getRadians());

    double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    // Base tolerance
    double maxTranslationJump = 0.5 + 0.35 * speed;
    double maxHeadingJump = Math.toRadians(20);

    // Strong multitag or multi-camera consensus may correct larger errors
    boolean strongVision = obs.isMultiTag && obs.avgTagDistance < 4.0;
    boolean hasSupport = supportCount >= 2;

    if (translationalError < maxTranslationJump && headingError < maxHeadingJump) {
      return true;
    }

    return strongVision || hasSupport;
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the
   * given poses.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for
       * factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate
       * pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the
       * simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    
    // Get the robot's current position for reference.
    Pose2d currentPose = swerveDrive.getPose();
    ChassisSpeeds speeds = swerveDrive.getRobotVelocity();

    List<VisionObservation> observations = new ArrayList<>();
    for (Cameras camera : Cameras.values()) {
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (!poseEst.isPresent()) continue; 

      EstimatedRobotPose estPos =  poseEst.get();
      Pose2d estPose2d = estPos.estimatedPose.toPose2d();
      
      int tagsUsed = estPos.targetsUsed == null ? 0 : estPos.targetsUsed.size();
      if (tagsUsed == 0) continue;

      double avgDist = averageTagDistance(estPos);
      double maxAmbiguity = maxAmbiguity(estPos);
      boolean isMultiTag = tagsUsed >= 2;
      Matrix<N3, N1> currStdDevs = camera.curStdDevs;

      observations.add(new VisionObservation(
        camera,
        estPos,
        estPose2d, 
        maxAmbiguity, 
        tagsUsed, 
        avgDist, 
        maxAmbiguity, 
        isMultiTag, 
        currStdDevs
        ));
    }

    if (observations.isEmpty()) return;

    // Reject inconsistent outliers using cross-camera agreement
    List<VisionObservation> filtered = filterByConsensus(observations);

    if (filtered.isEmpty()) {
      filtered = observations.stream()
          .filter(o -> o.isMultiTag)
          .toList();
    }

    if (filtered.isEmpty()) {
      return;
    }

    // Update the robot's position using the valid position observations.
    for (VisionObservation obs : filtered) {
      if (shouldAcceptEnabledMeasurement(obs, currentPose, speeds, filtered.size())) {
        swerveDrive.addVisionMeasurement(obs.pose, obs.timestamp, obs.stdDevs);
      }
    } 
  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   * <li>No Pose Estimates could be generated</li>
   * <li>The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and
   *         targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera) {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation()) {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est -> debugField
              .getObject("VisionEstimation")
              .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    }
    return poseEst;
  }

  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id) {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList) {
      if (result.hasTargets()) {
        for (PhotonTrackedTarget i : result.getTargets()) {
          if (i.getFiducialId() == id) {
            return i;
          }
        }
      }
    }
    return target;

  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim() {
    return visionSim;
  }

  /**
   * Open up the photon vision camera streams on the localhost, assumes running
   * photon vision on localhost.
   */
  private void openSimCameraViews() {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
      // try
      // {
      // Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
      // Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      // } catch (IOException | URISyntaxException e)
      // {
      // e.printStackTrace();
      // }
    }
  }

  /**
   * Update the {@link Field2d} to include tracked targets/
   */
  public void updateVisionField() {

    List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
    for (Cameras c : Cameras.values()) {
      if (!c.resultsList.isEmpty()) {
        PhotonPipelineResult latest = c.resultsList.get(0);
        if (latest.hasTargets()) {
          targets.addAll(latest.targets);
        }
      }
    }

    List<Pose2d> poses = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (fieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
        Pose2d targetPose = fieldLayout.getTagPose(target.getFiducialId()).get().toPose2d();
        poses.add(targetPose);
      }
    }

    field2d.getObject("tracked targets").setPoses(poses);
  }

}
