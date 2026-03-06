package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

/**
 * Limelight subsystem for vision-based AprilTag detection and pose estimation.
 * 
 * This subsystem wraps LimelightHelpers to provide a clean interface for:
 * - Detecting and tracking AprilTags
 * - Computing robot pose relative to specific tags
 * - Managing distance and angle measurements for alignment tasks
 * 
 * The subsystem is designed to be independent of drivetrain implementation,
 * allowing it to persist across drivetrain generation/replacement cycles.
 */
public class LimelightSubsystem6237 extends SubsystemBase {
    private final String limelightName;
    private boolean limelightConnected = false;
    
    // Continuous tracking of hub distance and heading
    private double lastHubDistance = -1.0;  // meters, -1 if no target
    private int lastHubTagID = -1;          // tag ID, -1 if no target
    private double lastHubTx = 0.0;         // horizontal angle to hub
    private boolean hasEverSeenHub = false;
    
    // Odometry-based tracking when vision is lost
    private edu.wpi.first.math.geometry.Pose2d lastKnownHubPose = null;  // Field position when hub was last seen
    private final frc.robot.subsystems.CommandSwerveDrivetrain drivetrain;

    /**
     * Constructs a LimelightSubsystem with the default Limelight name.
     * Uses empty string "" which corresponds to "limelight" on the network.
     */
    public LimelightSubsystem6237() {
        this("", null);
    }
    
    /**
     * Constructs a LimelightSubsystem with drivetrain for odometry-based distance updates.
     * @param drivetrain The swerve drivetrain for odometry
     */
    public LimelightSubsystem6237(frc.robot.subsystems.CommandSwerveDrivetrain drivetrain) {
        this("", drivetrain);
    }

    /**
     * Constructs a LimelightSubsystem with a specific Limelight name and optional drivetrain.
     * @param name The network name of the Limelight (e.g., "limelight-front", "limelight")
     * @param drivetrain The swerve drivetrain for odometry (can be null)
     */
    public LimelightSubsystem6237(String name, frc.robot.subsystems.CommandSwerveDrivetrain drivetrain) {
        this.limelightName = name;
        this.drivetrain = drivetrain;
        
        // Set Limelight to AprilTag pipeline
        // This ensures the Limelight is configured for fiducial detection
        LimelightHelpers.setPipelineIndex(limelightName, frc.robot.Constants.Limelight.kAprilTagPipelineIndex);
        
        // Set LED mode to pipeline default
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    }

    @Override
    public void periodic() {
        // Check Limelight connection status periodically
        updateConnectionStatus();
        
        // Continuously track hub distance and angle in the background
        updateHubTracking();
        
        // Update SmartDashboard with current status
        updateDashboard();
    }
    
    /**
     * Continuously updates hub tracking information in the background.
     * This runs every robot loop to keep distance/angle data fresh.
     */
    private void updateHubTracking() {
        // Use simple visible tag ID method (reads directly from NetworkTables)
        int visibleTagID = getVisibleTagID();
        
        // Debug outputs
        SmartDashboard.putBoolean("Limelight/DEBUG Has Valid Target", hasValidTarget());
        SmartDashboard.putNumber("Limelight/DEBUG Visible Tag ID", visibleTagID);
        SmartDashboard.putNumber("Limelight/DEBUG TID from NetworkTables", LimelightHelpers.getFiducialID(limelightName));
        SmartDashboard.putNumber("Limelight/DEBUG TY angle", LimelightHelpers.getTY(limelightName));
        SmartDashboard.putNumber("Limelight/DEBUG TX angle", LimelightHelpers.getTX(limelightName));
        
        // Check if we see a hub tag
        if (hasValidTarget() && (visibleTagID == frc.robot.Constants.Auto.kRedHubAprilTagID || 
                                 visibleTagID == frc.robot.Constants.Auto.kBlueHubAprilTagID)) {
            // Update tracking data
            lastHubTagID = visibleTagID;
            lastHubTx = getTxToTag(visibleTagID);
            hasEverSeenHub = true;
            
            // Calculate distance using simple method as fallback
            double simpleDistance = getSimpleDistance();
            double complexDistance = getDistanceToTag(visibleTagID);
            
            // Debug distance calculations
            SmartDashboard.putNumber("Limelight/DEBUG Simple Distance", simpleDistance);
            SmartDashboard.putNumber("Limelight/DEBUG Complex Distance", complexDistance);
            
            // Use whichever method works
            lastHubDistance = complexDistance > 0 ? complexDistance : simpleDistance;
            
            SmartDashboard.putNumber("Limelight/DEBUG Last Hub Distance", lastHubDistance);
            
            // Store robot pose when hub is visible (for odometry fallback)
            if (drivetrain != null && lastHubDistance > 0) {
                // Store the hub's position based on robot pose and distance
                edu.wpi.first.math.geometry.Pose2d robotPose = drivetrain.getState().Pose;
                double robotHeading = robotPose.getRotation().getRadians();
                double angleToHub = robotHeading + Math.toRadians(lastHubTx);
                
                // Calculate hub position
                double hubX = robotPose.getX() + lastHubDistance * Math.cos(angleToHub);
                double hubY = robotPose.getY() + lastHubDistance * Math.sin(angleToHub);
                lastKnownHubPose = new edu.wpi.first.math.geometry.Pose2d(
                    hubX, hubY, new edu.wpi.first.math.geometry.Rotation2d());
            }
            
            // Update SmartDashboard with all hub tracking data
            if (lastHubDistance > 0) {
                SmartDashboard.putNumber("Limelight/Hub Distance (m)", lastHubDistance);
                SmartDashboard.putNumber("Limelight/Hub Tag ID", lastHubTagID);
                SmartDashboard.putNumber("Limelight/Hub TX (deg)", lastHubTx);
                SmartDashboard.putBoolean("Limelight/Hub Visible", true);
                SmartDashboard.putString("Limelight/Hub Status", "Tracking Tag " + lastHubTagID);
                SmartDashboard.putString("Limelight/Distance Source", "Vision");
                
                // Also output in PrepareToFire format for compatibility
                SmartDashboard.putNumber("PrepareToFire/Distance To Hub (m)", lastHubDistance);
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", true);
                SmartDashboard.putNumber("PrepareToFire/Hub Tag ID", lastHubTagID);
            } else {
                SmartDashboard.putString("Limelight/Hub Status", "Tag visible but distance calc failed");
            }
        } else {
            // No hub visible - use odometry if available
            SmartDashboard.putBoolean("Limelight/Hub Visible", false);
            
            // If we have drivetrain and have seen hub before, calculate distance from odometry
            if (drivetrain != null && hasEverSeenHub && lastKnownHubPose != null) {
                edu.wpi.first.math.geometry.Pose2d robotPose = drivetrain.getState().Pose;
                double odometryDistance = Math.hypot(
                    robotPose.getX() - lastKnownHubPose.getX(),
                    robotPose.getY() - lastKnownHubPose.getY()
                );
                
                // Update distance with odometry calculation
                lastHubDistance = odometryDistance;
                
                // Calculate TX from odometry
                double angleToHub = Math.atan2(
                    lastKnownHubPose.getY() - robotPose.getY(),
                    lastKnownHubPose.getX() - robotPose.getX()
                );
                double robotHeading = robotPose.getRotation().getRadians();
                lastHubTx = Math.toDegrees(angleToHub - robotHeading);
                
                // Normalize to -180 to 180
                while (lastHubTx > 180) lastHubTx -= 360;
                while (lastHubTx < -180) lastHubTx += 360;
                
                // Update dashboard with odometry-based tracking
                SmartDashboard.putNumber("Limelight/Hub Distance (m)", lastHubDistance);
                SmartDashboard.putNumber("Limelight/Hub TX (deg)", lastHubTx);
                SmartDashboard.putString("Limelight/Hub Status", "Odometry Tracking (last: Tag " + lastHubTagID + ")");
                SmartDashboard.putString("Limelight/Distance Source", "Odometry");
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", true);
                SmartDashboard.putNumber("PrepareToFire/Distance To Hub (m)", lastHubDistance);
            } else {
                // No vision and no odometry fallback available
                SmartDashboard.putBoolean("PrepareToFire/Target Detected", false);
                if (hasEverSeenHub) {
                    SmartDashboard.putString("Limelight/Hub Status", "Lost (last: Tag " + lastHubTagID + ")");
                } else {
                    SmartDashboard.putString("Limelight/Hub Status", "Searching for hub tags");
                }
                SmartDashboard.putString("Limelight/Distance Source", "None");
            }
        }
    }

    /**
     * Checks if the Limelight is currently connected and receiving frames.
     * @return true if Limelight is connected, false otherwise
     */
    private void updateConnectionStatus() {
        // Limelight is considered connected if we have valid target data
        // This is a basic check - you can refine based on your needs
        limelightConnected = hasValidTarget();
    }

    /**
     * Updates SmartDashboard with current Limelight status and detection data.
     */
    private void updateDashboard() {
        SmartDashboard.putBoolean("Limelight/Connected", limelightConnected);
        SmartDashboard.putBoolean("Limelight/Has Target", hasValidTarget());
        SmartDashboard.putBoolean("Limelight/Any AprilTag Detected", getDetectedFiducialCount() > 0);
        SmartDashboard.putNumber("Limelight/Fiducials Detected", getDetectedFiducialCount());
        SmartDashboard.putNumber("Limelight/Current Pipeline", LimelightHelpers.getCurrentPipelineIndex(limelightName));
        
        // Debug: Show raw NetworkTables values
        SmartDashboard.putBoolean("Limelight/DEBUG TV", LimelightHelpers.getTV(limelightName));
        SmartDashboard.putNumber("Limelight/DEBUG TID", LimelightHelpers.getFiducialID(limelightName));
    }

    // ======================== TARGET DETECTION ========================

    /**
     * Checks if the Limelight has a valid target currently.
     * @return true if a target is detected, false otherwise
     */
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Gets all detected fiducials (AprilTags) from the Limelight's JSON output.
     * These contain full pose information relative to the robot.
     * @return Array of LimelightTarget_Fiducial objects
     */
    public LimelightTarget_Fiducial[] getDetectedFiducials() {
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        if (results == null) {
            return new LimelightTarget_Fiducial[0];
        }
        LimelightTarget_Fiducial[] fiducials = results.targets_Fiducials;
        return fiducials != null ? fiducials : new LimelightTarget_Fiducial[0];
    }

    /**
     * Gets the number of AprilTag fiducials currently detected by the Limelight.
     * @return Number of detected fiducials
     */
    public int getDetectedFiducialCount() {
        return getDetectedFiducials().length;
    }

    /**
     * Checks if a specific AprilTag ID is currently visible.
     * @param tagId The ID of the AprilTag to check
     * @return true if the tag is detected, false otherwise
     */
    public boolean isTagVisible(int tagId) {
        for (LimelightTarget_Fiducial fiducial : getDetectedFiducials()) {
            if ((int)fiducial.fiducialID == tagId) {
                return true;
            }
        }
        return false;
    }
    
    // ======================== HUB TRACKING (BACKGROUND) ========================
    
    /**
     * Gets the last tracked distance to the hub.
     * This is updated continuously in the background by periodic().
     * @return Distance to hub in meters, or -1.0 if no hub has been seen
     */
    public double getLastHubDistance() {
        return lastHubDistance;
    }
    
    /**
     * Gets the last tracked hub tag ID.
     * @return Hub tag ID (10 or 26), or -1 if no hub has been seen
     */
    public int getLastHubTagID() {
        return lastHubTagID;
    }
    
    /**
     * Gets the last tracked horizontal angle to the hub.
     * @return TX angle in degrees, or 0.0 if no hub has been seen
     */
    public double getLastHubTx() {
        return lastHubTx;
    }
    
    /**
     * Checks if the hub is currently visible.
     * @return true if a hub tag is currently visible, false otherwise
     */
    public boolean isHubCurrentlyVisible() {
        int visibleTagID = getBestHubTagID();
        return hasValidTarget() && (visibleTagID == frc.robot.Constants.Auto.kRedHubAprilTagID || 
                                     visibleTagID == frc.robot.Constants.Auto.kBlueHubAprilTagID);
    }
    
    /**
     * Checks if the hub has ever been seen since robot startup.
     * @return true if hub has been detected at least once, false otherwise
     */
    public boolean hasEverSeenHub() {
        return hasEverSeenHub;
    }

    // ======================== POSE ESTIMATION ========================

    /**
     * Gets the robot's pose relative to a specific AprilTag.
     * 
     * This returns the 3D pose of the robot as measured by the Limelight
     * relative to the specified tag's coordinate frame.
     * 
     * @param tagId The ID of the AprilTag
     * @return Pose3d representing robot pose relative to tag, or null if tag not visible
     */
    public Pose3d getRobotPoseRelativeToTag(int tagId) {
        if (!isTagVisible(tagId)) {
            return null;
        }

        for (LimelightTarget_Fiducial fiducial : getDetectedFiducials()) {
            if ((int)fiducial.fiducialID == tagId) {
                // getRobotPose_TargetSpace() returns the robot pose in the target's coordinate frame
                return fiducial.getRobotPose_TargetSpace();
            }
        }
        return null;
    }

    /**
     * Gets the 2D robot pose relative to a specific AprilTag (ignoring Z, roll, pitch).
     * 
     * @param tagId The ID of the AprilTag
     * @return Pose2d representing robot's X, Y, and rotation relative to tag, or null if not visible
     */
    public Pose2d getRobotPose2dRelativeToTag(int tagId) {
        if (!isTagVisible(tagId)) {
            return null;
        }

        for (LimelightTarget_Fiducial fiducial : getDetectedFiducials()) {
            if ((int)fiducial.fiducialID == tagId) {
                return fiducial.getRobotPose_TargetSpace2D();
            }
        }
        return null;
    }

    /**
     * Gets the 2D robot pose relative to a specific AprilTag (simplified).
     * Uses only X, Y, and Yaw from the Limelight's pose measurement.
     * 
     * @param tagId The ID of the AprilTag
     * @return Pose2d representing simplified robot pose, or null if tag not visible
     */
    public Pose2d getSimplifiedPose2dRelativeToTag(int tagId) {
        Pose3d pose3d = getRobotPoseRelativeToTag(tagId);
        if (pose3d == null) {
            return null;
        }
        double x = pose3d.getX();
        double y = pose3d.getY();
        double yaw = pose3d.getRotation().getZ();
        return new Pose2d(x, y, new Rotation2d(yaw));
    }

    // ======================== DISTANCE & ANGLE MEASUREMENTS ========================

    /**
     * Gets the straight-line distance from the robot to a specific AprilTag.
     * 
     * @param tagId The ID of the AprilTag
     * @return Distance in meters, or -1 if tag not visible
     */
    public double getDistanceToTag(int tagId) {
        Pose3d pose = getRobotPoseRelativeToTag(tagId);
        if (pose == null) {
            return -1;
        }
        // Distance is the magnitude of the translation vector
        return Math.sqrt(
            pose.getX() * pose.getX() +
            pose.getY() * pose.getY() +
            pose.getZ() * pose.getZ()
        );
    }

    /**
     * Gets the horizontal (X-Y plane) distance from the robot to a specific AprilTag.
     * This is useful for alignment tasks that don't care about vertical distance.
     * 
     * @param tagId The ID of the AprilTag
     * @return Horizontal distance in meters, or -1 if tag not visible
     */
    public double getHorizontalDistanceToTag(int tagId) {
        Pose3d pose = getRobotPoseRelativeToTag(tagId);
        if (pose == null) {
            return -1;
        }
        return Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY());
    }

    /**
     * Gets the angle (yaw) from the robot to a specific AprilTag in the robot's frame.
     * Positive angles are counter-clockwise when viewed from above.
     * 
     * @param tagId The ID of the AprilTag
     * @return Angle in radians, or 0 if tag not visible
     */
    public double getAngleToTag(int tagId) {
        Pose3d pose = getRobotPoseRelativeToTag(tagId);
        if (pose == null) {
            return 0;
        }
        // The angle in the X-Y plane
        return Math.atan2(pose.getY(), pose.getX());
    }

    /**
     * Gets the bearing angle from the robot's forward direction to the target tag.
     * This is the yaw rotation of the tag in the robot's frame.
     * 
     * @param tagId The ID of the AprilTag
     * @return Rotation2d representing the tag's heading, or null if not visible
     */
    public Rotation2d getTagHeading(int tagId) {
        Pose3d pose = getRobotPoseRelativeToTag(tagId);
        if (pose == null) {
            return null;
        }
        return new Rotation2d(pose.getRotation().getZ());
    }

    // ======================== ALIGNMENT SUPPORT ========================

    /**
     * Computes the error required to align the robot to a specific distance and angle from a tag.
     * 
     * This is useful for building alignment commands - it returns both the distance error
     * and the angle error needed to reach the desired pose.
     * 
     * @param tagId The ID of the AprilTag
     * @param desiredDistance Desired distance to maintain from the tag (meters)
     * @param desiredAngle Desired angle relative to the tag (radians)
     * @return AlignmentError object containing distance and angle errors, or null if tag not visible
     */
    public AlignmentError calculateAlignmentError(int tagId, double desiredDistance, double desiredAngle) {
        double actualDistance = getHorizontalDistanceToTag(tagId);
        double actualAngle = getAngleToTag(tagId);

        if (actualDistance < 0) {
            return null; // Tag not visible
        }

        return new AlignmentError(
            actualDistance - desiredDistance,  // positive = too close, negative = too far
            actualAngle - desiredAngle          // angular error in radians
        );
    }

    /**
     * Checks if the robot is within acceptable tolerance of a desired pose relative to a tag.
     * 
     * @param tagId The ID of the AprilTag
     * @param desiredDistance Desired distance (meters)
     * @param distanceTolerance Acceptable distance error (meters)
     * @param desiredAngle Desired angle (radians)
     * @param angleTolerance Acceptable angle error (radians)
     * @return true if within both tolerances, false otherwise
     */
    public boolean isAligned(int tagId, double desiredDistance, double distanceTolerance,
                            double desiredAngle, double angleTolerance) {
        AlignmentError error = calculateAlignmentError(tagId, desiredDistance, desiredAngle);
        if (error == null) {
            return false;
        }
        return Math.abs(error.distanceError) <= distanceTolerance &&
               Math.abs(error.angleError) <= angleTolerance;
    }

    // ======================== RAW DATA ACCESS ========================

    /**
     * Gets raw TX value (horizontal offset to target in degrees).
     * @return TX value in degrees
     */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    /**
     * Gets raw TY value (vertical offset to target in degrees).
     * @return TY value in degrees
     */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /**
     * Gets raw TA value (target area as percentage of image).
     * @return TA value (0-100)
     */
    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    /**
     * Gets the current active pipeline index.
     * @return Pipeline index (0-9)
     */
    public double getCurrentPipeline() {
        return LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    /**
     * Sets the active pipeline.
     * @param pipelineIndex The pipeline index to activate (0-9)
     */
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    /**
     * Gets the pipeline latency in milliseconds.
     * @return Latency in milliseconds
     */
    public double getPipelineLatency() {
        return LimelightHelpers.getLatency_Pipeline(limelightName);
    }

    /**
     * Gets the capture latency in milliseconds.
     * @return Latency in milliseconds
     */
    public double getCaptureLatency() {
        return LimelightHelpers.getLatency_Capture(limelightName);
    }

    // ======================== SIMPLE DISTANCE CALCULATION ========================
    
    /**
     * Gets a simple distance estimate using basic trigonometry.
     * This is a fallback method when full 3D pose isn't available.
     * 
     * REQUIRES: Camera height and angle must be configured in Constants
     * 
     * @return Distance in meters, or -1 if no target
     */
    public double getSimpleDistance() {
        if (!hasValidTarget()) {
            return -1;
        }
        
        // Get the vertical angle to target (ty)
        double ty = LimelightHelpers.getTY(limelightName);
        
        // TODO: Add camera mounting height and angle to Constants
        // For now, use placeholder values
        // Distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
        double cameraHeightMeters = 0.5; // PLACEHOLDER - measure your camera height
        double cameraAngleDegrees = 25.0; // PLACEHOLDER - measure your camera angle
        double targetHeightMeters = 1.5;  // PLACEHOLDER - measure AprilTag height
        
        double angleToTargetDegrees = cameraAngleDegrees + ty;
        double angleToTargetRadians = Math.toRadians(angleToTargetDegrees);
        
        double distance = (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToTargetRadians);
        
        return distance > 0 ? distance : -1;
    }
    
    /**
     * Gets the currently visible AprilTag ID using simple NetworkTables access.
     * @return The fiducial ID, or -1 if no target visible
     */
    public int getVisibleTagID() {
        if (!hasValidTarget()) {
            return -1;
        }
        return (int)LimelightHelpers.getFiducialID(limelightName);
    }
    
    /**
     * Gets the best hub tag ID to use, prioritizing center tags (10 and 26).
     * If multiple tags are visible, prefers the one closest to center of vision (smallest tx).
     * 
     * @return The preferred hub tag ID (10 or 26), or -1 if no hub tag visible
     */
    public int getBestHubTagID() {
        LimelightTarget_Fiducial[] fiducials = getDetectedFiducials();
        
        if (fiducials.length == 0) {
            return -1;
        }
        
        // Look for hub tags (10 or 26)
        LimelightTarget_Fiducial bestTag = null;
        double bestTx = Double.MAX_VALUE; // Smallest tx (closest to center) is best
        
        for (LimelightTarget_Fiducial fiducial : fiducials) {
            int tagId = (int)fiducial.fiducialID;
            
            // Only consider hub tags
            if (tagId == frc.robot.Constants.Auto.kRedHubAprilTagID || 
                tagId == frc.robot.Constants.Auto.kBlueHubAprilTagID) {
                
                double tx = Math.abs(fiducial.tx); // Use absolute value - closest to center
                
                if (bestTag == null || tx < bestTx) {
                    bestTag = fiducial;
                    bestTx = tx;
                }
            }
        }
        
        return bestTag != null ? (int)bestTag.fiducialID : -1;
    }
    
    /**
     * Gets the horizontal angle (tx) to a specific tag.
     * Positive is right, negative is left.
     * 
     * @param tagId The AprilTag ID
     * @return Angle in degrees, or 0 if tag not visible
     */
    public double getTxToTag(int tagId) {
        LimelightTarget_Fiducial[] fiducials = getDetectedFiducials();
        
        for (LimelightTarget_Fiducial fiducial : fiducials) {
            if ((int)fiducial.fiducialID == tagId) {
                return fiducial.tx;
            }
        }
        
        return 0;
    }

    // ======================== HELPER CLASS ========================

    /**
     * Represents alignment error between current and desired pose relative to a tag.
     */
    public static class AlignmentError {
        public final double distanceError;  // meters (positive = too close, negative = too far)
        public final double angleError;     // radians

        public AlignmentError(double distanceError, double angleError) {
            this.distanceError = distanceError;
            this.angleError = angleError;
        }

        @Override
        public String toString() {
            return String.format("AlignmentError{distance=%.3fm, angle=%.3frad}", distanceError, angleError);
        }
    }
}
