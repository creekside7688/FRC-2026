package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLookup {
    // distance from hub (inches, for now), RPM, angle (degrees)
    // RPM values are dummy values for now
    public static final double[][] LookupTable = {
        {220, 3500, 60.45}, // MAXIMUM
        {200, 3381.27, 61.13},
        {180, 3258.27, 61.92},
        {160, 3131.416, 62.86},
        {140, 3000.81, 64},
        {120, 2866.6, 65.417},
        {100, 2729.59, 67.2},
        {80, 2591.85, 69.54},
        {60, 2457.64, 72.69},
        {50, 2394.45, 74.685} // MINIMUM
    };

    public static InterpolatingDoubleTreeMap distanceAngleTable = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap distanceRPMTable = new InterpolatingDoubleTreeMap();

    public static void initializeTable() {
        for (int i = 0; i < LookupTable.length; i++) {
            distanceAngleTable.put(LookupTable[i][0], LookupTable[i][2]);
        }
        for (int i = 0; i < LookupTable.length; i++) {
            distanceRPMTable.put(LookupTable[i][0], LookupTable[i][1]);
        }
    }

    public static final boolean validShot(double distance) {
        return distance <= 220 && distance >= 50;
    }

    public static final double lookupRPM(double distance) {
        distance *= 39.3701; // meters to inches
        if (validShot(distance)) {
            return distanceRPMTable.get(distance);
        } else {
            return 500;
        }
    }

    public static final double lookupAngle(double distance) {
        distance *= 39.3701; // meters to inches
        if (validShot(distance)) {
            return distanceAngleTable.get(distance);
        } else {
            return 60;
        }
    }
}
