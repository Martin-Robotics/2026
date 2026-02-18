package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
public class LimelightSubsystem6237 implements Subsystem {
    private final String limelightName;
    private boolean limelightConnected = false;

    /**
     * Constructs a LimelightSubsystem with the default Limelight name.
     * Uses empty string "" which corresponds to "limelight" on the network.
     */
    public LimelightSubsystem6237() {
        this("");
    }

    /**
     * Constructs a LimelightSubsystem with a specific Limelight name.
     * @param name The network name of the Limelight (e.g., "limelight-front", "limelight")
     */
    public LimelightSubsystem6237(String name) {
        this.limelightName = name;
    }

    @Override
    public void periodic() {
        // Check Limelight connection status periodically
        updateConnectionStatus();
        
        // Update SmartDashboard with current status
        updateDashboard();
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
        SmartDashboard.putNumber("Limelight/Fiducials Detected", getDetectedFiducialCount());
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
