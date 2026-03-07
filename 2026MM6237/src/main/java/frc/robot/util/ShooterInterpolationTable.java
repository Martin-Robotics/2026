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
        // ============================================================
        // TODO: Replace these placeholder values with your tuned values!
        
        rpmTable.put(1.5, 2500.0);   // Close shot - lower RPM
        rpmTable.put(2.5, 3000.0);   // Medium-close
        rpmTable.put(3.5, 3500.0);   // Medium
        rpmTable.put(4.5, 4000.0);   // Medium-far
        rpmTable.put(5.5, 4500.0);   // Far
        rpmTable.put(6.5, 5000.0);   // Very far
        
        // ============================================================
        // HOOD POSITION DATA POINTS
        // Format: hoodTable.put(distance_meters, hood_position_0_to_1);
        // Lower position = flatter trajectory (for close shots)
        // Higher position = steeper trajectory (for far shots)
        // ============================================================
        // TODO: Replace these placeholder values with your tuned values!
        
        hoodTable.put(1.5, 0.3);     // Close - flatter
        hoodTable.put(2.5, 0.4);     // Medium-close
        hoodTable.put(3.5, 0.5);     // Medium
        hoodTable.put(4.5, 0.6);     // Medium-far
        hoodTable.put(5.5, 0.7);     // Far - steeper
        hoodTable.put(6.5, 0.75);    // Very far
    }
    
    /**
     * Get the interpolated RPM for a given distance.
     * @param distanceMeters Distance to hub in meters
     * @return Recommended shooter RPM
     */
    public static double getRPMForDistance(double distanceMeters) {
        // Clamp to valid range
        distanceMeters = Math.max(1.5, Math.min(6.5, distanceMeters));
        return rpmTable.get(distanceMeters);
    }
    
    /**
     * Get the interpolated hood position for a given distance.
     * @param distanceMeters Distance to hub in meters
     * @return Recommended hood position (0.0 - 1.0)
     */
    public static double getHoodPositionForDistance(double distanceMeters) {
        // Clamp to valid range
        distanceMeters = Math.max(1.5, Math.min(6.5, distanceMeters));
        return hoodTable.get(distanceMeters);
    }
    
    /**
     * Get minimum supported distance for interpolation.
     */
    public static double getMinDistance() {
        return 1.5;
    }
    
    /**
     * Get maximum supported distance for interpolation.
     */
    public static double getMaxDistance() {
        return 6.5;
    }
}
