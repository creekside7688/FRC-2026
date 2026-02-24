package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation3d;

public class ShooterLookup {
    // distance from hub (inches, for now), RPM, angle (degrees)
    // RPM values are dummy values for now

    // all in M/s for the middle value
    // ignore this, this is for velocities
    public static final double[][] VelocitiesLookupTable = {
        {220, 8.6123, 60.45}, // MAXIMUM
        {210, 8.46653, 60.77},
        {200, 8.31858, 61.13},
        {190, 8.16839, 61.51},
        {180, 8.0159, 61.92},
        {170, 7.86107, 62.37},
        {160, 7.70389, 62.86},
        {150, 7.54436, 63.41},
        {140, 7.38251, 64},
        {130, 7.21846, 64.67},
        {120, 2866.6, 65.417},
        {100, 2729.59, 67.2},
        {80, 2591.85, 69.54},
        {60, 2457.64, 72.69},
        {50, 2394.45, 74.685} // MINIMUM
    };

    public static InterpolatingDoubleTreeMap distanceAngleTable = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap distanceRPMTable = new InterpolatingDoubleTreeMap();

    public static void initializeTable() {

        final double[][] LookupTable = { // <= THIS ONE FOR RPMS!!!!
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

    public static final double CalculationDelayChangedAngle(
            double x, double y, double dxdt, double dydt, double calculationLatency) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation3d hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;

        double distx = hubPose.getX() - x;
        double disty = hubPose.getY() - y;

        double dx = dxdt * calculationLatency;
        double dy = dydt * calculationLatency;

        if (distx == 0 && disty == 0) return 0;

        double answer = (((dy * distx) - (dx * disty))) / (Math.pow((distx), 2) + Math.pow((disty), 2));
        return (answer * 180 / Math.PI);
    }

    public static final double CalculationDelayChangedDistance(
            double x, double y, double dxdt, double dydt, double calculationLatency) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation3d hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;

        double distx = hubPose.getX() - x;
        double disty = hubPose.getY() - y;

        double dx = dxdt * calculationLatency;
        double dy = dydt * calculationLatency;

        if (distx == 0 && disty == 0) return 0;

        double answer = (((distx * dx) + (disty * dy)) / (Math.sqrt(Math.pow(distx, 2) + Math.pow(disty, 2))));
        return answer; // in meters
    }

    // not done
    public static final void RecurssiveFOTDistance(double x, double y, double dxdt, double dydt, double angle) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation3d hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;
        double distx, disty;
        if (alliance == Alliance.Red) {
            distx = x - hubPose.getX();
            disty = y - hubPose.getY();
        } else {
            distx = hubPose.getX() - x;
            disty = hubPose.getY() - y;
        }

        double totalDistance = Math.sqrt(Math.pow(distx, 2) + Math.pow(disty, 2));

        double Vx = 0;
    }
}
