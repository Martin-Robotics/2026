package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem6237;

/**
 * Static-placement trim calibrator for competition field use.
 * 
 * HOW IT WORKS:
 * 1. Place robot at a known calibration position on the field (see CALIBRATION_FIELD_GUIDE.md)
 * 2. Select the matching position from the SmartDashboard chooser
 * 3. Press "Calc Trim" button on the dashboard
 * 4. The command computes the EXPECTED angle from the known pose to the hub center
 * 5. It reads the ACTUAL corrected TX (before trim) from the Limelight
 * 6. The difference = the recommended trim for whichever tag is visible
 * 7. Results are published to SmartDashboard and can be accepted with one click
 * 
 * No robot movement required — just static placement at a known position.
 */
public class CalibrateTrimCommand {

    private static final String DASH_PREFIX = "TrimCal/";

    // =====================================================================
    // CALIBRATION POSITIONS
    // =====================================================================
    // Each position is designed so the robot sees exactly ONE hub tag.
    // Robot heading is always aimed at the hub center for that position.
    //
    // DESIGN PRINCIPLE: All distances are measured from the HUB STRUCTURE
    // or field center line — never more than ~2.5m (~8 ft). A single
    // pool noodle (~5 ft / 1.5m) or piece of wood can mark most positions.
    //
    // Reference points:
    //   Blue hub front face: X = 4.022 m
    //   Red  hub front face: X = 12.519 m
    //   Blue hub side faces: X = 4.270 m, Y = 3.431 (low) / 4.638 (high)
    //   Red  hub side faces: X = 12.271 m, Y = 3.431 (low) / 4.638 (high)
    //   Field center line:   Y = 4.035 m (also both hub center Y)
    //
    // Robot: 0.9m x 0.9m, center-to-bumper = 0.45m
    // =====================================================================

    /**
     * A named calibration position with known robot pose.
     */
    public static class CalPosition {
        public final String name;
        public final int expectedTagID;
        public final double robotCenterX;  // meters, field coords
        public final double robotCenterY;  // meters, field coords
        public final double robotHeadingDeg; // degrees, field-relative (0 = facing +X)

        /** Short human-readable placement description. */
        public final String placementNotes;

        public CalPosition(String name, int expectedTagID,
                           double robotCenterX, double robotCenterY, double robotHeadingDeg,
                           String placementNotes) {
            this.name = name;
            this.expectedTagID = expectedTagID;
            this.robotCenterX = robotCenterX;
            this.robotCenterY = robotCenterY;
            this.robotHeadingDeg = robotHeadingDeg;
            this.placementNotes = placementNotes;
        }
    }

    // =====================================================================
    // BLUE HUB CALIBRATION POSITIONS
    // =====================================================================
    // Blue hub center: (4.146, 4.035)
    // Blue hub front face at X = 4.022, side faces at X = 4.270
    //
    // Front tags (25, 26): Robot directly in front, ~2m from front face
    // Side tags (24, 27):  Robot beside the hub, ~2m from side face
    // =====================================================================

    // Position B1: See Tag 26 (Blue front center)
    // Front bumper 2.0m from hub front face → robot center X = 4.022 + 2.0 + 0.45 = 6.472
    // Centered on field Y → robot center Y = 4.035
    // Heading: 180° (facing Blue wall, straight at hub)
    private static final CalPosition BLUE_TAG26 = new CalPosition(
        "B1: Tag 26 (Blue front center)",
        26,
        6.472, 4.035,
        180.0,
        "2.0m (6ft 7in) from hub front face, centered on hub. Facing Blue wall."
    );

    // Position B2: See Tag 25 (Blue front off-center, at Y=4.390)
    // Same X as B1. Shift robot 0.7m toward bottom wall so tag 25 dominates over 26.
    // Robot center Y = 4.035 - 0.7 = 3.335
    // Heading: angled toward tag 25 area ≈ 168°
    private static final CalPosition BLUE_TAG25 = new CalPosition(
        "B2: Tag 25 (Blue front off-ctr)",
        25,
        6.472, 3.335,
        168.0,
        "2.0m (6ft 7in) from hub front face, shifted 0.7m (2ft 4in) toward bottom wall. Angled slightly toward top."
    );

    // Position B3: See Tag 27 (Blue side low-Y, at (4.270, 3.431), faces -Y)
    // Robot must be on +Y side (above tag), looking down toward it.
    // Place robot 2.0m from side face in Y direction:
    //   Robot center Y = 3.431 + 2.0 + 0.45 = 5.881 → but that's far from wall... 
    //   Actually: tag faces -Y, robot needs to be BELOW it? No — faces -Y means visible from -Y side.
    //   Robot needs to be at LOWER Y than the tag, looking UP at it.
    //   Wait: tag faces -Y means the tag normal points toward -Y (bottom wall).
    //   So the tag is visible to something that is on the -Y side (below) looking toward +Y.
    //   But our robot would need to face the tag, meaning face +Y... that doesn't aim at the hub.
    //   
    //   Rethink: the robot needs to see this tag while generally facing the hub.
    //   Tag 27 is on the BOTTOM side of the blue hub (Y=3.431). Its face points toward -Y.
    //   A robot that is below and in front of the hub (X > 4.270, Y < 3.431) can see this tag
    //   if the Limelight FOV catches it while aiming at the hub area.
    //   
    //   Better approach: robot at X ≈ 5.5, Y ≈ 2.0, aimed at hub center.
    //   Distance from hub side face: sqrt((5.5-4.270)^2 + (2.0-3.431)^2) = sqrt(1.51 + 2.05) = 1.89m
    //   Heading to hub center: atan2(4.035-2.0, 4.146-5.5) = atan2(2.035, -1.354) = 124°
    //   
    //   Placement: 1.4m (4ft 7in) from hub side face toward bottom wall (in Y)
    //              + 1.2m (4ft) in front of hub side face (in X)
    //   → Can use an L of two pool noodle cuts.
    private static final CalPosition BLUE_TAG27 = new CalPosition(
        "B3: Tag 27 (Blue side low-Y)",
        27,
        5.5, 2.0,
        124.0,
        "L-shape: 1.2m (4ft) in front of + 1.4m (4ft 7in) below hub bottom-side corner. Aim toward hub."
    );

    // Position B4: See Tag 24 (Blue side high-Y, at (4.270, 4.638), faces +Y)
    // Mirror of B3 on the other side. Robot above and in front of hub.
    // Robot at X ≈ 5.5, Y ≈ 6.07 (mirrored: 4.035 + (4.035-2.0) = 6.07)
    // Heading to hub center: atan2(4.035-6.07, 4.146-5.5) = atan2(-2.035, -1.354) = 236° (or -124°)
    private static final CalPosition BLUE_TAG24 = new CalPosition(
        "B4: Tag 24 (Blue side high-Y)",
        24,
        5.5, 6.07,
        236.0,
        "L-shape: 1.2m (4ft) in front of + 1.4m (4ft 7in) above hub top-side corner. Aim toward hub."
    );

    // =====================================================================
    // RED HUB CALIBRATION POSITIONS
    // =====================================================================
    // Red hub center: (12.395, 4.035)
    // Red hub front face at X = 12.519, side faces at X = 12.271
    //
    // Mirror of Blue positions about field center X = 8.2705
    // =====================================================================

    // Position R1: See Tag 10 (Red front center)
    // Front bumper 2.0m from hub front face → robot center X = 12.519 - 2.0 - 0.45 = 10.069
    // Centered on hub Y → robot center Y = 4.035
    // Heading: 0° (facing Red wall, straight at hub)
    private static final CalPosition RED_TAG10 = new CalPosition(
        "R1: Tag 10 (Red front center)",
        10,
        10.069, 4.035,
        0.0,
        "2.0m (6ft 7in) from hub front face, centered on hub. Facing Red wall."
    );

    // Position R2: See Tag 9 (Red front off-center, at Y=3.679)
    // Same X as R1. Shift robot 0.7m toward top wall so tag 9 (which is below center) dominates.
    // Robot center Y = 4.035 + 0.7 = 4.735
    // Heading: angled toward tag 9 area ≈ -12°
    private static final CalPosition RED_TAG9 = new CalPosition(
        "R2: Tag 9 (Red front off-ctr)",
        9,
        10.069, 4.735,
        -12.0,
        "2.0m (6ft 7in) from hub front face, shifted 0.7m (2ft 4in) toward top wall. Angled slightly toward bottom."
    );

    // Position R3: See Tag 8 (Red side low-Y, at (12.271, 3.431), faces -Y)
    // Mirror of B3. Robot above and in front of Red hub.
    // Robot at X ≈ 11.041, Y ≈ 2.0
    // Heading to hub center: atan2(4.035-2.0, 12.395-11.041) = atan2(2.035, 1.354) = 56°
    private static final CalPosition RED_TAG8 = new CalPosition(
        "R3: Tag 8 (Red side low-Y)",
        8,
        11.041, 2.0,
        56.0,
        "L-shape: 1.2m (4ft) in front of + 1.4m (4ft 7in) below hub bottom-side corner. Aim toward hub."
    );

    // Position R4: See Tag 11 (Red side high-Y, at (12.271, 4.638), faces +Y)
    // Mirror of B4. Robot above and in front of Red hub.
    // Robot at X ≈ 11.041, Y ≈ 6.07
    // Heading to hub center: atan2(4.035-6.07, 12.395-11.041) = atan2(-2.035, 1.354) = -56°
    private static final CalPosition RED_TAG11 = new CalPosition(
        "R4: Tag 11 (Red side high-Y)",
        11,
        11.041, 6.07,
        -56.0,
        "L-shape: 1.2m (4ft) in front of + 1.4m (4ft 7in) above hub top-side corner. Aim toward hub."
    );

    /** All calibration positions, in order. */
    public static final CalPosition[] ALL_POSITIONS = {
        BLUE_TAG26, BLUE_TAG25, BLUE_TAG27, BLUE_TAG24,
        RED_TAG10,  RED_TAG9,   RED_TAG8,   RED_TAG11
    };

    // =====================================================================
    // Dashboard integration
    // =====================================================================

    private static SendableChooser<CalPosition> positionChooser;

    /**
     * Call from RobotContainer.initializeDashboard() to set up the calibration UI.
     * Creates:
     * - A chooser for selecting the calibration position
     * - Dashboard entries for results
     * - "Calc Trim" boolean trigger
     */
    public static void initDashboard() {
        positionChooser = new SendableChooser<>();
        positionChooser.setDefaultOption(ALL_POSITIONS[0].name, ALL_POSITIONS[0]);
        for (int i = 1; i < ALL_POSITIONS.length; i++) {
            positionChooser.addOption(ALL_POSITIONS[i].name, ALL_POSITIONS[i]);
        }
        SmartDashboard.putData(DASH_PREFIX + "Position", positionChooser);
        SmartDashboard.putBoolean(DASH_PREFIX + "Calc Trim", false);
        SmartDashboard.putString(DASH_PREFIX + "Status", "Ready — select position and press Calc Trim");
        SmartDashboard.putNumber(DASH_PREFIX + "Result Tag ID", 0);
        SmartDashboard.putNumber(DASH_PREFIX + "Expected TX (deg)", 0);
        SmartDashboard.putNumber(DASH_PREFIX + "Actual TX (deg)", 0);
        SmartDashboard.putNumber(DASH_PREFIX + "Recommended Trim", 0);
        SmartDashboard.putNumber(DASH_PREFIX + "Current Trim", 0);
    }

    /**
     * Creates an InstantCommand that performs one trim calibration snapshot.
     * Bind this to the "Calc Trim" dashboard button or a controller button.
     */
    public static InstantCommand createCalibrationCommand(
            LimelightSubsystem6237 limelight,
            CommandSwerveDrivetrain drivetrain) {
        return new InstantCommand(() -> runCalibration(limelight, drivetrain));
    }

    /**
     * Call this periodically (e.g., from Robot.teleopPeriodic or RobotContainer)
     * to poll the dashboard button. When "Calc Trim" is pressed, runs calibration.
     */
    public static void pollDashboardTrigger(
            LimelightSubsystem6237 limelight,
            CommandSwerveDrivetrain drivetrain) {
        boolean trigger = SmartDashboard.getBoolean(DASH_PREFIX + "Calc Trim", false);
        if (trigger) {
            SmartDashboard.putBoolean(DASH_PREFIX + "Calc Trim", false); // reset
            runCalibration(limelight, drivetrain);
        }
    }

    // =====================================================================
    // Core calibration logic
    // =====================================================================

    /**
     * Runs a single trim calibration snapshot.
     * 
     * Algorithm:
     * 1. Get the selected calibration position (known robot pose)
     * 2. Read the currently visible tag ID and raw TX from Limelight
     * 3. Compute the EXPECTED TX from the known pose to the hub center
     *    using pure geometry (same math as computeHubCenterCorrection, but
     *    using the KNOWN pose instead of odometry)
     * 4. Compute the corrected TX the Limelight reports (WITH the current trim)
     * 5. Back out the current trim to get the "actual corrected TX before trim"
     * 6. Recommended trim = expected TX − actual TX before trim
     */
    private static void runCalibration(
            LimelightSubsystem6237 limelight,
            CommandSwerveDrivetrain drivetrain) {
        
        CalPosition pos = positionChooser.getSelected();
        if (pos == null) {
            SmartDashboard.putString(DASH_PREFIX + "Status", "ERROR: No position selected");
            return;
        }

        // Check if Limelight sees a hub tag
        if (!limelight.isHubCurrentlyVisible()) {
            SmartDashboard.putString(DASH_PREFIX + "Status",
                "ERROR: No hub tag visible! Verify robot placement at " + pos.name);
            return;
        }

        int visibleTagID = limelight.getLastHubTagID();
        if (visibleTagID < 0) {
            SmartDashboard.putString(DASH_PREFIX + "Status", "ERROR: No tag ID");
            return;
        }

        // Warn if wrong tag is visible (but still compute — might be useful)
        boolean wrongTag = (visibleTagID != pos.expectedTagID);

        // --- Get actual Limelight data ---
        double rawTx = limelight.getLastHubTx();              // raw TX to TAG
        double hubCenterTxWithTrim = limelight.getHubCenterTx(); // corrected TX WITH current trim
        double distanceToTag = limelight.getLastHubDistance();  // distance to TAG

        // Read current trim from SmartDashboard
        double currentTrim = SmartDashboard.getNumber(
            "TargetTuning/Trim Tag " + visibleTagID,
            Constants.HubGeometry.getDefaultTrim(visibleTagID));

        // Back out the current trim to get "corrected TX before trim"
        double actualCorrectedTxNoTrim = hubCenterTxWithTrim - currentTrim;

        // --- Compute EXPECTED TX from known pose to hub center ---
        double[] hubCenter = Constants.HubGeometry.getHubCenter(visibleTagID);
        if (hubCenter == null) {
            SmartDashboard.putString(DASH_PREFIX + "Status", "ERROR: Unknown tag " + visibleTagID);
            return;
        }

        // Vector from known robot position to hub center in field coords
        double dxField = hubCenter[0] - pos.robotCenterX;
        double dyField = hubCenter[1] - pos.robotCenterY;

        // Angle from robot to hub center in field coords
        double fieldAngleToHub = Math.atan2(dyField, dxField); // radians, field frame

        // Robot heading in radians
        double robotHeadingRad = Math.toRadians(pos.robotHeadingDeg);

        // Expected TX = angle to hub in ROBOT frame
        // Robot-relative angle: field angle minus robot heading
        double expectedTxRad = fieldAngleToHub - robotHeadingRad;
        // Normalize to -pi..pi
        while (expectedTxRad > Math.PI) expectedTxRad -= 2 * Math.PI;
        while (expectedTxRad < -Math.PI) expectedTxRad += 2 * Math.PI;
        double expectedTxDeg = Math.toDegrees(expectedTxRad);

        // --- Recommended trim ---
        // We want: actualCorrectedTxNoTrim + recommendedTrim = expectedTxDeg
        // (when aimed correctly, hub center TX should be ~0, so we want the trim
        //  to make the system report 0 when actually pointing at hub center)
        // 
        // Actually: the trim is added to make the robot aim correctly.
        // If expected TX = 2° but Limelight reports 5° (before trim), the robot
        // would over-rotate by 3°. We need trim = 2° - 5° = -3° to compensate.
        double recommendedTrim = expectedTxDeg - actualCorrectedTxNoTrim;

        // --- Publish results ---
        String status;
        if (wrongTag) {
            status = String.format("WARNING: Expected Tag %d but saw Tag %d — results may still be useful",
                pos.expectedTagID, visibleTagID);
        } else {
            status = String.format("OK: Tag %d — Recommended trim: %.1f° (current: %.1f°)",
                visibleTagID, recommendedTrim, currentTrim);
        }

        SmartDashboard.putString(DASH_PREFIX + "Status", status);
        SmartDashboard.putNumber(DASH_PREFIX + "Result Tag ID", visibleTagID);
        SmartDashboard.putNumber(DASH_PREFIX + "Expected TX (deg)", 
            Math.round(expectedTxDeg * 10.0) / 10.0);
        SmartDashboard.putNumber(DASH_PREFIX + "Actual TX (deg)", 
            Math.round(actualCorrectedTxNoTrim * 10.0) / 10.0);
        SmartDashboard.putNumber(DASH_PREFIX + "Recommended Trim", 
            Math.round(recommendedTrim * 10.0) / 10.0);
        SmartDashboard.putNumber(DASH_PREFIX + "Current Trim", currentTrim);
        SmartDashboard.putNumber(DASH_PREFIX + "Distance to Tag (m)",
            Math.round(distanceToTag * 100.0) / 100.0);
        SmartDashboard.putNumber(DASH_PREFIX + "Raw LL TX (deg)",
            Math.round(rawTx * 10.0) / 10.0);

        // Also show what the auto-apply would do
        SmartDashboard.putBoolean(DASH_PREFIX + "Apply Trim", false);

        System.out.printf("[TrimCal] Pos=%s  Tag=%d  Expected=%.1f°  Actual=%.1f°  Recommend=%.1f°  (current=%.1f°)%n",
            pos.name, visibleTagID, expectedTxDeg, actualCorrectedTxNoTrim, recommendedTrim, currentTrim);
    }

    /**
     * Poll for the "Apply Trim" button and write the recommended trim
     * to the live SmartDashboard tuning entry for the last calibrated tag.
     */
    public static void pollApplyTrigger() {
        boolean apply = SmartDashboard.getBoolean(DASH_PREFIX + "Apply Trim", false);
        if (apply) {
            SmartDashboard.putBoolean(DASH_PREFIX + "Apply Trim", false);
            int tagID = (int) SmartDashboard.getNumber(DASH_PREFIX + "Result Tag ID", 0);
            double recommended = SmartDashboard.getNumber(DASH_PREFIX + "Recommended Trim", 0);
            if (tagID > 0) {
                String key = "TargetTuning/Trim Tag " + tagID;
                SmartDashboard.putNumber(key, recommended);
                SmartDashboard.putString(DASH_PREFIX + "Status",
                    String.format("APPLIED: Tag %d trim set to %.1f°", tagID, recommended));
                System.out.printf("[TrimCal] Applied trim %.1f° to Tag %d%n", recommended, tagID);
            }
        }
    }
}
