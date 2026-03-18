package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
/**
 * Computes the average distance from the camera to each AprilTag used in a pose estimate.
 *
 * <p>This is used as a rough quality metric for the estimate. In general, estimates built
 * from closer tags are more trustworthy than estimates built from distant tags, since distance
 * tends to amplify vision noise and pose uncertainty.
 *
 * @param est the estimated robot pose containing the list of AprilTag targets used
 * @return the average camera-to-tag distance in meters, or {@link Double#POSITIVE_INFINITY}
 *     if no targets were used
 */
private double averageTagDistance(EstimatedRobotPose est) {
  // If no tags contributed to this estimate, treat the distance as effectively unusable.
  if (est.targetsUsed == null || est.targetsUsed.isEmpty()) return Double.POSITIVE_INFINITY;

  double sum = 0.0;

  /*
   * Sum the Euclidean distance from the camera to every tag that contributed
   * to this pose estimate.
   */
  for (var target : est.targetsUsed) {
    sum += target.getBestCameraToTarget().getTranslation().getNorm();
  }

  // Return the mean distance across all contributing tags.
  return sum / est.targetsUsed.size();
}

/**
 * Finds the highest pose ambiguity among all AprilTags used in a pose estimate.
 *
 * <p>Pose ambiguity is a PhotonVision-provided measure describing how uncertain a tag-based
 * pose solve is. A larger ambiguity generally indicates a less trustworthy estimate.
 * This method uses the worst contributing tag as a conservative quality metric.
 *
 * @param est the estimated robot pose containing the list of AprilTag targets used
 * @return the maximum ambiguity value across all used tags, or {@code 1.0} if no targets were used
 */
private double maxAmbiguity(EstimatedRobotPose est) {
  // If no tags were used, return a pessimistic ambiguity value.
  if (est.targetsUsed == null || est.targetsUsed.isEmpty()) return 1.0;

  double max = 0.0;

  /*
   * Track the worst ambiguity among all tags contributing to this estimate.
   * Using the maximum ambiguity is intentionally conservative.
   */
  for (var target : est.targetsUsed) {
    max = Math.max(max, target.getPoseAmbiguity());
  }

  return max;
}

/**
 * Determines whether two robot poses agree closely enough to be considered consistent.
 *
 * <p>Agreement is based on both translation and rotation:
 * <ul>
 *   <li>The XY distance between the poses must be less than {@code maxXY}</li>
 *   <li>The angular difference between the poses must be less than {@code maxThetaRad}</li>
 * </ul>
 *
 * @param posA the first pose
 * @param posB the second pose
 * @param maxXY the maximum allowed translational difference in meters
 * @param maxThetaRad the maximum allowed rotational difference in radians
 * @return {@code true} if the poses are within both tolerances, otherwise {@code false}
 */
private boolean agrees(Pose2d posA, Pose2d posB, double maxXY, double maxThetaRad) {
  return posA.getTranslation().getDistance(posB.getTranslation()) < maxXY
      && Math.abs(posA.getRotation().minus(posB.getRotation()).getRadians()) < maxThetaRad;
}

/**
 * Filters a list of vision observations by requiring cross-camera agreement or strong multitag support.
 *
 * <p>An observation is accepted if either:
 * <ul>
 *   <li>At least one other camera reports a sufficiently similar pose, or</li>
 *   <li>The observation is a strong multitag estimate</li>
 * </ul>
 *
 * <p>This helps reject outliers from single-camera failures while still allowing good multitag
 * estimates to pass through even if no other camera currently agrees.
 *
 * @param obs the raw list of vision observations
 * @return a filtered list containing only observations that passed consensus checks
 */
private List<VisionObservation> filterByConsensus(List<VisionObservation> obs) {
  List<VisionObservation> accepted = new ArrayList<>();

  /*
   * Compare every observation against every other observation.
   * If another camera agrees with it, it gains support and is more likely to be valid.
   */
  for (VisionObservation obsA : obs) {
    int agreements = 0;

    for (VisionObservation obsB : obs) {
      // Skip comparing an observation to itself.
      if (obsA == obsB) continue;

      /*
       * Count how many other observations are spatially consistent with this one.
       * The thresholds here define what "agreement" means across cameras.
       */
      if (agrees(obsA.pose, obsB.pose, 0.6, Math.toRadians(15))) {
        agreements++;
      }
    }

    /*
     * Keep the observation if:
     * 1. At least one other camera agrees with it, indicating cross-camera consensus, or
     * 2. It is a strong multitag result, which is generally more trustworthy on its own.
     */
    if (agreements > 0 || (obsA.isMultiTag && obsA.tagCount >= 2 && obsA.avgTagDistance < 4.0)) {
      accepted.add(obsA);
    }
  }

  return accepted;
}

/**
 * Determines whether a vision measurement should be accepted given the current robot state.
 *
 * <p>Robot motion and odometry updates are actively occurring, so blindly accepting large pose
 * jumps can destabilize localization. This method allows:
 * <ul>
 *   <li>Measurements close to the current pose estimate</li>
 *   <li>Larger corrections if the vision is especially strong</li>
 *   <li>Larger corrections if multiple observations support the measurement</li>
 * </ul>
 *
 * @param obs the vision observation being considered
 * @param currentPose the robot's current estimated pose from odometry / pose estimation
 * @param speeds the robot's current chassis speeds
 * @param supportCount the number of accepted observations supporting the current vision update cycle
 * @return {@code true} if the measurement should be applied, otherwise {@code false}
 */
private boolean shouldAcceptEnabledMeasurement(
    VisionObservation obs,
    Pose2d currentPose,
    ChassisSpeeds speeds,
    int supportCount) {

  // Compute translational and angular error between the current estimate and vision.
  double translationalError =
      currentPose.getTranslation().getDistance(obs.pose.getTranslation());
  double headingError =
      Math.abs(currentPose.getRotation().minus(obs.pose.getRotation()).getRadians());

  // Use planar speed magnitude to scale allowable positional correction.
  double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

  /*
   * Base tolerance:
   * - Allow larger translational jumps when the robot is moving faster, since odometry can drift
   *   more under motion.
   * - Keep heading jump tolerance fixed for now.
   */
  double maxTranslationJump = 0.5 + 0.35 * speed;
  double maxHeadingJump = Math.toRadians(20);

  /*
   * Define conditions under which a larger-than-normal correction is acceptable:
   * - strongVision: multitag solution with reasonably close tags
   * - hasSupport: multiple accepted observations this cycle
   */
  boolean strongVision = obs.isMultiTag && obs.avgTagDistance < 4.0;
  boolean hasSupport = supportCount >= 2;

  // Accept small, ordinary corrections immediately.
  if (translationalError < maxTranslationJump && headingError < maxHeadingJump) {
    return true;
  }

  /*
   * Otherwise, only accept the correction if the vision data is especially trustworthy
   * or multiple observations support it.
   */
  return strongVision || hasSupport;
}

/**
 * Updates the robot pose estimator with valid vision measurements from all configured cameras.
 *
 * <p>This method performs the following high-level steps:
 * <ol>
 *   <li>Updates the vision simulator during simulation</li>
 *   <li>Collects pose estimates from each camera</li>
 *   <li>Builds {@link VisionObservation} objects with quality metadata</li>
 *   <li>Filters out inconsistent observations using cross-camera consensus</li>
 *   <li>Falls back to multitag-only observations if consensus fails</li>
 *   <li>Applies accepted measurements to the swerve drive pose estimator</li>
 * </ol>
 *
 * <p>The goal is to make vision updates robust against bad individual camera measurements while
 * still allowing strong observations to correct odometry drift.
 *
 * @param swerveDrive the {@link SwerveDrive} instance whose pose estimator should be updated
 */
public void updatePoseEstimation(SwerveDrive swerveDrive) {
  if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent()) {
    /*
     * In simulation, the drivetrain pose used by the simulator represents the "ground truth"
     * position of the robot.
     *
     * Simulated odometry may still include effects such as skidding or drift, meaning the pose
     * estimator's internal odometry can deviate from truth just like on a real robot.
     *
     * Since vision exists specifically to correct odometry drift, the vision simulator should
     * be updated using the true simulated robot pose rather than the potentially drifted estimate.
     */
    visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
  }

  // Read the robot's current estimated pose and velocity for gating future measurements.
  Pose2d currentPose = swerveDrive.getPose();
  ChassisSpeeds speeds = swerveDrive.getRobotVelocity();

  List<VisionObservation> observations = new ArrayList<>();

  /*
   * Query every configured camera for a global pose estimate.
   * Each valid estimate is converted into a VisionObservation containing
   * pose data plus metadata used for filtering and trust evaluation.
   */
  for (Cameras camera : Cameras.values()) {
    Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);

    // Skip cameras that do not currently have a valid estimate.
    if (!poseEst.isPresent()) continue;

    EstimatedRobotPose estPos = poseEst.get();
    Pose2d estPose2d = estPos.estimatedPose.toPose2d();

    // Determine how many tags contributed to this estimate.
    int tagsUsed = estPos.targetsUsed == null ? 0 : estPos.targetsUsed.size();

    // Ignore estimates that were not built from any tags.
    if (tagsUsed == 0) continue;

    // Compute quality metrics used later for filtering and acceptance decisions.
    double avgDist = averageTagDistance(estPos);
    double maxTagAmbiguity = maxAmbiguity(estPos);
    boolean isMultiTag = tagsUsed >= 2;
    Matrix<N3, N1> currStdDevs = camera.curStdDevs;

    // Reject highly ambiguous estimates before they enter the consensus pipeline.
    if (maxAmbiguity > maximumAmbiguity) {
      continue;
    }

    /*
     * Store a normalized observation record for later filtering.
     * This keeps the later pipeline cleaner by collecting all relevant
     * metadata in one place.
     */
    observations.add(new VisionObservation(
      camera,
      estPos,
      estPose2d,
      estPos.timestampSeconds,
      tagsUsed,
      avgDist,
      maxTagAmbiguity,
      isMultiTag,
      currStdDevs
    ));
  }

  // Nothing to do if no camera produced a usable observation.
  if (observations.isEmpty()) return;

  // First-pass rejection of outliers using cross-camera agreement.
  List<VisionObservation> filtered = filterByConsensus(observations);

  if (filtered.isEmpty()) {
    /*
     * If no observations achieved consensus, fall back to multitag-only observations.
     * This gives strong single-camera multitag estimates a chance to still correct pose.
     */
    filtered = observations.stream()
        .filter(o -> o.isMultiTag)
        .toList();
  }

  // If nothing survived filtering or fallback, skip this update cycle entirely.
  if (filtered.isEmpty()) {
    return;
  }

  /*
   * Feed accepted vision measurements into the drivetrain pose estimator.
   * Each measurement is individually gated against current pose and robot motion.
   */
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
