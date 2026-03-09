package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Shooter interpolation tables for distance-based RPM and hood position lookup.
 * 
 * TUNING INSTRUCTIONS:
 * 1. Use ShooterTuningCommand to find optimal RPM and hood position at each distance
 * 2. Record your findings in the data points below
 * 3. The interpolation will automatically blend between your tested distances
 * 
 * Data points format: put(distanceMeters, value)
 */
public class ShooterInterpolationTable {
    
    // RPM lookup table - interpolates between tested distances
    private static final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    
    // Hood position lookup table - interpolates between tested distances
    private static final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    
    static {
        // ============================================================
        // SHOOTER RPM DATA POINTS
        // Format: rpmTable.put(distance_meters, rpm);
        // Keyed to ACTUAL Limelight-reported distances (not physical tape measure)
        // 
        // CALIBRATION NOTE (2026-03-07):
        // These values were tuned with RIGHT SHOOTER ONLY (correct roller wheel installed).
        // Left and middle shooters are awaiting parts. RPM values will likely need
        // adjustment once all three wheels are installed and the 1.25x right multiplier
        // is factored across all motors.
        // ============================================================
        
        rpmTable.put(1.8, 2700.0);   // ~1.5m physical, Limelight reads 1.8m
        rpmTable.put(2.9, 3000.0);   // ~2.5m physical, Limelight reads 2.9m
        rpmTable.put(3.8, 3400.0);   // ~3.5m physical, Limelight reads 3.8m
        rpmTable.put(4.7, 3800.0);   // ~4.5m physical, Limelight reads 4.7m
        rpmTable.put(5.4, 4100.0);   // ~5.0m physical, Limelight reads 5.4m
        
        // ============================================================
        // HOOD POSITION DATA POINTS
        // Format: hoodTable.put(distance_meters, hood_position_0_to_1);
        // Lower position = flatter trajectory (for close shots)
        // Higher position = steeper trajectory (for far shots)
        // ============================================================
        
        hoodTable.put(1.8, 0.25);    // ~1.5m physical
        hoodTable.put(2.9, 0.40);    // ~2.5m physical
        hoodTable.put(3.8, 0.50);    // ~3.5m physical
        hoodTable.put(4.7, 0.50);    // ~4.5m physical (same as 3.5m)
        hoodTable.put(5.4, 0.62);    // ~5.0m physical
    }
    
    /**
     * Get the interpolated RPM for a given distance.
     * @param distanceMeters Distance to hub in meters
     * @return Recommended shooter RPM
     */
    public static double getRPMForDistance(double distanceMeters) {
        // Clamp to valid range (Limelight-reported distances)
        distanceMeters = Math.max(1.8, Math.min(5.4, distanceMeters));
        return rpmTable.get(distanceMeters);
    }
    
    /**
     * Get the interpolated hood position for a given distance.
     * @param distanceMeters Distance to hub in meters (Limelight-reported)
     * @return Recommended hood position (0.0 - 1.0)
     */
    public static double getHoodPositionForDistance(double distanceMeters) {
        // Clamp to valid range (Limelight-reported distances)
        distanceMeters = Math.max(1.8, Math.min(5.4, distanceMeters));
        return hoodTable.get(distanceMeters);
    }
    
    /**
     * Get minimum supported distance for interpolation.
     */
    public static double getMinDistance() {
        return 1.8;
    }
    
    /**
     * Get maximum supported distance for interpolation.
     */
    public static double getMaxDistance() {
        return 5.4;
    }
}
