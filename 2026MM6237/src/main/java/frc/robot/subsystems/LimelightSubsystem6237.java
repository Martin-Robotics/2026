package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private double lastHubDistance = -1.0;  // meters to TAG, -1 if no target
    private int lastHubTagID = -1;          // tag ID, -1 if no target
    private double lastHubTx = 0.0;         // horizontal angle to TAG (raw)
    private double lastHubCenterTx = 0.0;   // horizontal angle to HUB CENTER (corrected)
    private double lastHubCenterDistance = -1.0; // distance to HUB CENTER (corrected)
    private boolean hasEverSeenHub = false;
    private boolean hubCurrentlyVisible = false;  // Updated each loop cycle
    
    // Tag lock hysteresis — prevents bouncing between adjacent tags (e.g., 25/26)
    // Currently disabled: simple direct tracking proved more reliable in testing
    // private int lockedTagID = -1;
    // private int framesWithoutLockedTag = 0;
    // private static final int LOCK_RELEASE_FRAMES = 15;
    
    // TX smoothing — low-pass filter to dampen frame-to-frame jitter
    // Currently disabled: added too much lag for our use case
    // private double smoothedHubCenterTx = 0.0;
    // private static final double SMOOTHING_ALPHA = 0.15;
    
    // Odometry-based tracking when vision is lost
    private edu.wpi.first.math.geometry.Pose2d lastKnownHubPose = null;  // Field position when hub was last seen
    private final frc.robot.subsystems.CommandSwerveDrivetrain drivetrain;

    // Target tuning dashboard state
    private boolean targetTuningEnabled = false;
    private static final String TT_PREFIX = "TargetTuning/";
    // Tag IDs for iteration
    private static final int[] ALL_HUB_TAG_IDS = {8, 9, 10, 11, 24, 25, 26, 27};

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
        // Single efficient update cycle - reads from Limelight once per loop
        updateHubTracking();
    }
    
    /**
     * Continuously updates hub tracking information in the background.
     * This runs every robot loop to keep distance/angle data fresh.
     * 
     * OPTIMIZED: Only reads from NetworkTables once per cycle, avoids JSON parsing.
     * Recognizes ALL hub tags (8-11 for Red, 24-27 for Blue) and computes a
     * corrected TX/distance aimed at the hub CENTER rather than the tag face.
     */
    private void updateHubTracking() {
        // === SINGLE READ FROM NETWORKTABLES ===
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        
        int primaryTagID = hasTarget ? (int)LimelightHelpers.getFiducialID(limelightName) : -1;
        double primaryTx = hasTarget ? LimelightHelpers.getTX(limelightName) : 0;
        double primaryTy = hasTarget ? LimelightHelpers.getTY(limelightName) : 0;
        
        // Cache connection status
        limelightConnected = hasTarget;
        
        // === CHECK IF THIS IS ANY HUB TAG (all 8 tags, both alliances) ===
        int hubTagID = -1;
        double hubTx = 0;
        double hubTy = 0;
        boolean foundHubTag = false;
        
        if (hasTarget && Constants.HubGeometry.isHubTag(primaryTagID)) {
            hubTagID = primaryTagID;
            hubTx = primaryTx;
            hubTy = primaryTy;
            foundHubTag = true;
        }
        
        // Cache whether hub is currently visible for external queries
        hubCurrentlyVisible = foundHubTag;
        
        if (foundHubTag) {
            // Update tracking data with raw tag data
            lastHubTagID = hubTagID;
            lastHubTx = hubTx;
            hasEverSeenHub = true;
            
            // Calculate distance to TAG using simple trigonometry
            lastHubDistance = calculateSimpleDistance(hubTy);
            
            // === COMPUTE CORRECTED TX AND DISTANCE TO HUB CENTER ===
            // Uses robot heading + tag field position + hub center position
            // to calculate where the hub center is relative to the robot
            computeHubCenterCorrection(hubTagID, hubTx, lastHubDistance);
            
            // Store hub CENTER field position for odometry fallback (not tag position)
            if (drivetrain != null && lastHubCenterDistance > 0) {
                double[] hubCenter = Constants.HubGeometry.getHubCenter(hubTagID);
                if (hubCenter != null) {
                    lastKnownHubPose = new Pose2d(
                        hubCenter[0], hubCenter[1], new Rotation2d());
                }
            }
            
            // Update SmartDashboard
            SmartDashboard.putNumber("Hub Distance (m)", lastHubCenterDistance);
            SmartDashboard.putNumber("Hub Tag ID", lastHubTagID);
            SmartDashboard.putBoolean("Hub Visible", true);
            
        } else {
            // No hub visible - use odometry fallback if available
            SmartDashboard.putBoolean("Hub Visible", false);
            
            if (drivetrain != null && hasEverSeenHub && lastKnownHubPose != null) {
                Pose2d robotPose = drivetrain.getState().Pose;
                
                // Calculate distance from odometry to hub CENTER
                lastHubCenterDistance = Math.hypot(
                    robotPose.getX() - lastKnownHubPose.getX(),
                    robotPose.getY() - lastKnownHubPose.getY()
                );
                lastHubDistance = lastHubCenterDistance; // Best estimate when vision lost
                
                // Calculate corrected TX from odometry to hub CENTER
                double angleToHub = Math.atan2(
                    lastKnownHubPose.getY() - robotPose.getY(),
                    lastKnownHubPose.getX() - robotPose.getX()
                );
                double robotHeading = robotPose.getRotation().getRadians();
                lastHubCenterTx = Math.toDegrees(angleToHub - robotHeading);
                lastHubTx = lastHubCenterTx; // Best estimate when vision lost
                
                // Normalize to -180 to 180
                while (lastHubCenterTx > 180) lastHubCenterTx -= 360;
                while (lastHubCenterTx < -180) lastHubCenterTx += 360;
                lastHubTx = lastHubCenterTx;
                
                SmartDashboard.putNumber("Limelight/Hub Distance (m)", lastHubCenterDistance);
            }
        }
    }
    
    /**
     * Computes the corrected TX angle and distance from the robot to the hub CENTER,
     * given raw Limelight data pointing at a specific tag on the hub face.
     * 
     * Uses the robot's heading from odometry to transform the known field-space offset
     * (tag position → hub center) into the robot's reference frame.
     * 
     * @param tagID The visible hub tag ID
     * @param rawTxDegrees The raw Limelight TX to the tag (degrees)
     * @param distanceToTag The computed distance to the tag (meters)
     */
    private void computeHubCenterCorrection(int tagID, double rawTxDegrees, double distanceToTag) {
        double[] tagFieldPos = Constants.HubGeometry.getTagPosition(tagID);
        double[] hubCenter = Constants.HubGeometry.getHubCenter(tagID);
        
        if (tagFieldPos == null || hubCenter == null || distanceToTag <= 0 || drivetrain == null) {
            // Can't compute correction — fall back to raw values
            lastHubCenterTx = rawTxDegrees;
            lastHubCenterDistance = distanceToTag;
            return;
        }
        
        // Offset from tag to hub center in field coordinates
        double offsetFieldX = hubCenter[0] - tagFieldPos[0];
        double offsetFieldY = hubCenter[1] - tagFieldPos[1];
        
        // Get robot heading to rotate field offset into robot-relative frame
        double robotHeading = drivetrain.getState().Pose.getRotation().getRadians();
        
        // Rotate offset into robot-relative coordinates
        // Robot frame: +X = forward, +Y = left
        double cosH = Math.cos(-robotHeading);
        double sinH = Math.sin(-robotHeading);
        double offsetRobotX = offsetFieldX * cosH - offsetFieldY * sinH;
        double offsetRobotY = offsetFieldX * sinH + offsetFieldY * cosH;
        
        // Tag position in robot-relative coordinates (from TX and distance)
        double rawTxRadians = Math.toRadians(rawTxDegrees);
        double tagRobotX = distanceToTag * Math.cos(rawTxRadians);
        double tagRobotY = distanceToTag * Math.sin(rawTxRadians);
        
        // Hub center in robot-relative coordinates
        double hubCenterRobotX = tagRobotX + offsetRobotX;
        double hubCenterRobotY = tagRobotY + offsetRobotY;
        
        // Corrected TX: angle from robot forward to hub center
        lastHubCenterTx = Math.toDegrees(Math.atan2(hubCenterRobotY, hubCenterRobotX));
        
        // Corrected distance: straight-line to hub center
        lastHubCenterDistance = Math.hypot(hubCenterRobotX, hubCenterRobotY);
        
        // Apply per-tag TX trim from SmartDashboard (live tunable)
        double trim = SmartDashboard.getNumber(TT_PREFIX + "Trim Tag " + tagID,
                Constants.HubGeometry.getDefaultTrim(tagID));
        double rawCorrectedTx = lastHubCenterTx;
        lastHubCenterTx += trim;
        
        // Publish diagnostics when tuning dashboard is active
        if (targetTuningEnabled) {
            SmartDashboard.putNumber(TT_PREFIX + "Active Tag", tagID);
            SmartDashboard.putNumber(TT_PREFIX + "Raw LL TX (deg)", rawTxDegrees);
            SmartDashboard.putNumber(TT_PREFIX + "Corrected TX (deg)", rawCorrectedTx);
            SmartDashboard.putNumber(TT_PREFIX + "Trim (deg)", trim);
            SmartDashboard.putNumber(TT_PREFIX + "Final TX (deg)", lastHubCenterTx);
            SmartDashboard.putNumber(TT_PREFIX + "Distance to Tag (m)", distanceToTag);
            SmartDashboard.putNumber(TT_PREFIX + "Distance to Hub (m)", lastHubCenterDistance);
        }
    }
    
    /**
     * Calculates distance using simple trigonometry from TY angle.
     * This avoids expensive JSON parsing of full pose data.
     * 
     * @param ty The vertical angle to target in degrees
     * @return Distance in meters, or -1 if invalid
     */
    private double calculateSimpleDistance(double ty) {
        // TODO: Update these values for your robot's camera mounting
        double cameraHeightMeters = frc.robot.Constants.Limelight.kCameraHeightMeters;
        double cameraAngleDegrees = frc.robot.Constants.Limelight.kCameraMountAngleDegrees;
        double targetHeightMeters = frc.robot.Constants.Limelight.kHubAprilTagHeightMeters;
        
        double angleToTargetRadians = Math.toRadians(cameraAngleDegrees + ty);
        
        if (Math.abs(angleToTargetRadians) < 0.01) {
            return -1; // Avoid division by near-zero
        }
        
        double distance = (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToTargetRadians);
        return distance > 0 ? distance : -1;
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
     * Gets the last tracked distance to the hub TAG face.
     * For shooting calculations, prefer {@link #getHubCenterDistance()}.
     * @return Distance to tag in meters, or -1.0 if no hub has been seen
     */
    public double getLastHubDistance() {
        return lastHubDistance;
    }
    
    /**
     * Gets the corrected distance to the hub CENTER (not the tag face).
     * This is the distance that should be used for shooter RPM / hood interpolation.
     * @return Distance to hub center in meters, or -1.0 if no hub has been seen
     */
    public double getHubCenterDistance() {
        return lastHubCenterDistance;
    }
    
    /**
     * Gets the last tracked hub tag ID.
     * @return Hub tag ID (8-11 for Red, 24-27 for Blue), or -1 if no hub has been seen
     */
    public int getLastHubTagID() {
        return lastHubTagID;
    }
    
    /**
     * Gets the raw horizontal angle to the hub TAG face.
     * For aiming, prefer {@link #getHubCenterTx()}.
     * @return TX angle in degrees, or 0.0 if no hub has been seen
     */
    public double getLastHubTx() {
        return lastHubTx;
    }
    
    /**
     * Gets the corrected horizontal angle to the hub CENTER (not the tag face).
     * This is the TX that should be used for aiming commands (PrepareToFire, AimAtHubWhileDriving).
     * @return Corrected TX angle in degrees, or 0.0 if no hub has been seen
     */
    public double getHubCenterTx() {
        return lastHubCenterTx;
    }
    
    /**
     * Gets the raw (unsmoothed) corrected TX to hub center. Same as getHubCenterTx().
     * @return Corrected TX angle in degrees
     */
    public double getRawHubCenterTx() {
        return lastHubCenterTx;
    }
    
    /**
     * Gets the currently locked tag ID for anti-oscillation.
     * Tag locking is currently disabled — returns the last seen hub tag ID instead.
     * @return The last seen hub tag ID, or -1 if none
     */
    public int getLockedTagID() {
        return lastHubTagID;
    }
    
    /**
     * Checks if the hub is currently visible.
     * Uses cached value from periodic() - no additional Limelight reads.
     * @return true if a hub tag is currently visible, false otherwise
     */
    public boolean isHubCurrentlyVisible() {
        return hubCurrentlyVisible;
    }

    /**
     * Checks if the robot is aimed at the hub within the tunable tx tolerance defined
     * in Constants.Limelight.kAimedAtHubTxTolerance.
     * Only returns true when a hub tag (10 or 26) is currently visible AND
     * the horizontal offset is within tolerance of center.
     * Use this for the "safe to fire" LED indicator instead of hasValidTarget().
     *
     * @return true if hub is visible and tx is within tolerance of 0
     */
    public boolean isAimedAtHub() {
        return hubCurrentlyVisible && Math.abs(lastHubTx) <= frc.robot.Constants.Limelight.kAimedAtHubTxTolerance;
    }
    
    /**
     * Checks if the hub has ever been seen since robot startup.
     * @return true if hub has been detected at least once, false otherwise
     */
    public boolean hasEverSeenHub() {
        return hasEverSeenHub;
    }

    // ======================== TARGET TUNING DASHBOARD ========================

    /**
     * Publishes per-tag TX trim controls and real-time diagnostic values to SmartDashboard
     * under the "TargetTuning/" prefix.
     * 
     * Trims are seeded from Constants.HubGeometry defaults (initially 0.0).
     * Adjust trims live on SmartDashboard, then copy final values back to Constants.
     * 
     * Published entries:
     *   TargetTuning/Trim Tag 8..11, 24..27   — editable TX offsets (degrees)
     *   TargetTuning/Active Tag               — which tag is currently driving aiming
     *   TargetTuning/Raw LL TX (deg)          — raw Limelight TX before any correction
     *   TargetTuning/Corrected TX (deg)       — after geometric hub-center correction
     *   TargetTuning/Trim (deg)               — the trim being applied for the active tag
     *   TargetTuning/Final TX (deg)           — corrected + trim (what the robot aims at)
     *   TargetTuning/Distance to Tag (m)      — raw distance to the tag face
     *   TargetTuning/Distance to Hub (m)      — corrected distance to hub center
     */
    public void initializeTargetTuningDashboard() {
        targetTuningEnabled = true;

        // Seed per-tag trim entries with defaults from Constants
        for (int tagID : ALL_HUB_TAG_IDS) {
            SmartDashboard.putNumber(TT_PREFIX + "Trim Tag " + tagID,
                    Constants.HubGeometry.getDefaultTrim(tagID));
        }

        // Seed diagnostic readouts with zeros
        SmartDashboard.putNumber(TT_PREFIX + "Active Tag", 0);
        SmartDashboard.putNumber(TT_PREFIX + "Raw LL TX (deg)", 0.0);
        SmartDashboard.putNumber(TT_PREFIX + "Corrected TX (deg)", 0.0);
        SmartDashboard.putNumber(TT_PREFIX + "Trim (deg)", 0.0);
        SmartDashboard.putNumber(TT_PREFIX + "Final TX (deg)", 0.0);
        SmartDashboard.putNumber(TT_PREFIX + "Distance to Tag (m)", 0.0);
        SmartDashboard.putNumber(TT_PREFIX + "Distance to Hub (m)", 0.0);
    }

    /**
     * Stops publishing diagnostic values to SmartDashboard (trim entries remain
     * so the robot still reads them — they just won't update in real time).
     * Call this to reduce dashboard clutter during competition.
     */
    public void removeTargetTuningDashboard() {
        targetTuningEnabled = false;
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
