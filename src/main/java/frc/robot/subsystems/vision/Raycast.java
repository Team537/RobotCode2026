package frc.robot.subsystems.vision;

import java.net.InetAddress;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import org.apache.commons.text.similarity.LevenshteinDistance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Constants.RaycastConstants;
import frc.robot.network.TCPSender;
import frc.robot.network.UDPReceiver;
import frc.robot.util.vision.detections.RobotDetection;

/**
 * 
 */
public class Raycast {
    
    // Networking
    private UDPReceiver<RobotDetection[]> robotReceiver;
    //private UDPReceiver<> ballReceiver;
    
    private TCPSender poseSender;

    // Suppliers
    private Supplier<Pose2d> getRobotPosSupplier;

    // Storage
    private final String RIO_IP;
    private final Field2d raycastField;
    private final FieldObject2d displayField;
    
    private LevenshteinDistance distanceMetric = LevenshteinDistance.getDefaultInstance();

    private final java.util.concurrent.atomic.AtomicReference<Pose2d> latestPose =
        new java.util.concurrent.atomic.AtomicReference<>(new Pose2d());

    // Runnables
    private ScheduledExecutorService scheduler; // Separate independent scheduler to ensure that loop overruns do not interfere with performance.
    private Runnable raycastPeriodic;

    // Flags
    private volatile boolean started = false;
    private volatile boolean setupFailed = false;

    // Singleton Instance.
    private static Raycast singleInstance;


    /*
     * -----------------------------------------------------
     * CREATION
     * -----------------------------------------------------
     */

    /**
     * Creates a new Raycast instance using the values provided in Constants.
     */
    private Raycast() {
        
        // Get constants.
        this.RIO_IP = getRioIP();
        this.raycastField = new Field2d();
        this.displayField = this.raycastField.getObject(RaycastConstants.RAYCAST_FIELD_NAME);

        // Setup networking clients.
        try {
            this.robotReceiver = new UDPReceiver<>(RaycastConstants.ROBOT_DETECTION_PORT, RobotDetection[].class);
            this.poseSender = new TCPSender(RaycastConstants.JETSON_IP, RaycastConstants.TCP_PORT);
        } catch (Exception e) {
            System.err.println("!!WARNING!! RIO COULDN'T CONNECT TO THE JETSON! ROBOT DETECTION FUNCTIONALITY WON'T WORK!!");
            this.setupFailed = true;
        }

        // Setup functionality for the periodic loop.
        this.scheduler = Executors.newScheduledThreadPool(1);
        this.raycastPeriodic = () -> {
            this.periodic();
        };
    }

    /**
     * Returns the singular Raycast instance.
     * 
     * @return The singular Raycast instance.
     */
    public static synchronized Raycast getInstance() {

        // Create a new Raycast instance if one has not yet been created.
        if (singleInstance == null) {
            singleInstance = new Raycast();
        }

        // Return the singular Raycast instance.
        return singleInstance;
    }


    /*
     * -----------------------------------------------------
     * CORE FUNCTIONALITY
     * -----------------------------------------------------
     */

    /**
     * Begins all network communication protocols & starts running the internal periodic.
     */
    public synchronized void start() {

        // Return if this operation has already been completed.
        if (this.started || this.setupFailed) {
            return;
        }

        // Toggle the started flag.
        this.started = true;

        // Start all networking instances.
        this.robotReceiver.start();

        // Start the periodic loop.
        this.scheduler.scheduleAtFixedRate(this.raycastPeriodic, 0, 20, TimeUnit.MILLISECONDS); // Runs the periodic ~50 times per second.
    }

    /**
     * Closes all networking connections to ensure resources are cleaned up properly.
     */
    public void close() {
        this.robotReceiver.close();
        this.poseSender.close();
    }

    /**
     * Periodic method that updates general functionality at set intervals.
     */
    private void periodic() {
        this.updateRobotPos();
        this.visualizeData();
    }


    /*
     * -----------------------------------------------------
     * HELPER METHODS
     * -----------------------------------------------------
     */

     /**
      * Attempts to get the RoboRIO's IP. Returns -1 if the IP cannot be found.

      * @return Returns the RoboRIO's IP or -1 if the Ip cannot be found.
      */
    private String getRioIP() {

        String hostIP = "-1";

         try {
            hostIP = InetAddress.getLocalHost().getHostAddress();
        } catch (Exception e) {
            System.out.println("Could not get IP Address: " + e.getMessage());
        }   

        return hostIP;
    }

    /**
     * TODO: Update to use UDP.
     */
    private void updateRobotPos() {

        // Do nothing if we cannot get the the robot's pos.
        if (this.getRobotPosSupplier == null) {
            System.err.println("!!WARNING!! RAYCAST WAS UNABLE TO UPDATE THE ROBOT'S POSITION! MAKE SURE YOU CALL setRobotPoseSupplier()!!");
            return;
        }

        // Get the latest robot pose.
        Pose2d robotPose2d = this.getRobotPosSupplier.get();
        Translation2d robotTranslation2d = robotPose2d.getTranslation();
        Rotation2d robotRotation2d = robotPose2d.getRotation();

        // Extract specific key parameters.
        double xMeters = robotTranslation2d.getX();
        double yMeters = robotTranslation2d.getY();
        double zMeters = 0; // We don't really need Z, so we'll assume it's zero. We may want to replace this with the hight of the camera to ensure that the returned heights are distance off the ground.

        double yawRadiance = robotRotation2d.getRadians();
    }


    /*
     * -----------------------------------------------------
     * SETTER METHODS
     * -----------------------------------------------------
     */

    /**
     * Updates the internally stored most recent robot pose. This is called in {@code DriveSubsystem} to help ensure thread safety.
     * 
     * @param pose The most recent, up-to-date robot pose.
     */
    public void publishRobotPose(Pose2d pose) {
        latestPose.set(pose);
    }

    /*
     * -----------------------------------------------------
     * GETTER METHODS
     * -----------------------------------------------------
     */

     /**
      * Returns the latest robot detection.

      * @return An array (RobotDetection[]) containing the latest robot detection. Expect ~100ms latency.
      */
    public RobotDetection[] getRobotDetections() {
        return this.robotReceiver.getLatest();
    }

    /**
     * Returns true if Raycast failed during startup.
     * 
     * @return True if Raycast failed during startup.
     */
    public boolean getSetupStatus() {
        return this.setupFailed;
    }

    /**
     * Returns the `RobotDetection` instance that is closest to the given team number. 
     * NOTE!! This will still return an instance by default even if there is no robot that would reasonably fit the given team number.
     * Make sure to tune {@code maxAcceptedScore} to prevent this from happening. (Try 1-2 to start).
     * 
     * @param teamNumber The team # of the robot that will be returned.
     * @param maxAcceptedScore The maximum Levenshtein Distance score that will be accepted as a valid robot.
     * 
     * @return An optional that may or may not contain the robot with the specified team number.
     */
    public Optional<RobotDetection> getRobot(int teamNumber, int maxAcceptedScore) {

        // Setup relevant parameters.
        String targetNumAsString = Integer.toString(teamNumber);
        RobotDetection closesDetection = null;
        int lowestScore = maxAcceptedScore;
        double highestConfidence = 0;

        for (RobotDetection robotDetection : this.getRobotDetections()) {
            int robotScore = distanceMetric.apply(targetNumAsString, Integer.toString(robotDetection.teamNumber));

            // If there is a tie, we use the more confident detection.
            if (robotScore < lowestScore || (robotScore == lowestScore && highestConfidence < robotDetection.teamNumberConfidence)) {
                closesDetection = robotDetection;
                lowestScore = robotScore;
                highestConfidence = robotDetection.teamNumberConfidence;
            }
        }

        return Optional.ofNullable(closesDetection);
    }


    /**
     * -----------------------------------------------------
     * TESTING / DEBUGGING / VISUALIZATION
     * -----------------------------------------------------
     */

    /**
     * Visualization method that displays detections on a Field2d object.
     */
    private void visualizeData() {

        // Get all current detections and format their positions to be displayable on the Field2d object. 
        RobotDetection[] currentDetections = this.getRobotDetections();
        List<Pose2d> robotPositions = new ArrayList<Pose2d>();

        for (RobotDetection robotDetection : currentDetections) {
            robotPositions.add(new Pose2d(robotDetection.getPoseAsTranslation2d(), Rotation2d.fromDegrees(0)));
        }

        // Display the robot positions on the field for debugging purposes.
        this.displayField.setPoses(robotPositions);
    }
}