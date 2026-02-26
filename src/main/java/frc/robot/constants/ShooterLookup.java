package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterLookup {
    // distance from hub (inches, for now), RPM, angle (degrees)
    // RPM values are dummy values for now

    // all in M/s for the middle value
    // ignore this, this is for velocities

    public static InterpolatingDoubleTreeMap distanceAngleTable = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap distanceRPMTable = new InterpolatingDoubleTreeMap();

    public static InterpolatingDoubleTreeMap distanceVelocitiesTable = new InterpolatingDoubleTreeMap();

    public static void initializeTable() {

        final double[][] VelocitiesLookupTable = {
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
            {120, 7.05237, 65.417},
            {110, 6.88433, 66.2556},
            {100, 6.7154, 67.2},
            {90, 6.54569, 68.29},
            {80, 6.376424, 69.545},
            {70, 6.209175, 70.99777},
            {60, 6.04623, 72.69},
            {50, 5.890913, 74.685} // MINIMUM
        };

        final double[][] LookupTable = { // <= THIS ONE FOR RPMS!!!!
            {240, 4440, 59.86},
            {220, 4355, 60.45}, // MAXIMUM
            {200, 4300, 61.13},
            {180, 4050, 61.92},
            {160, 3930, 62.86},
            {140, 3790, 64},
            {120, 3570, 65.417},
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

        for (int i = 0; i < VelocitiesLookupTable.length; i++) {
            distanceVelocitiesTable.put(VelocitiesLookupTable[i][0], VelocitiesLookupTable[i][1]);
        }
    }

    public static final boolean validShot(double distance) {
        return distance <= 240 && distance >= 50;
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
    public static final double RecurssiveFOTDistance(
            double x, double y, double dxdt, double dydt, double hoodAngle, double angle, double distance) {

        double newAngle = angle;
        double newDistance = distance;

        for (int i = 0; i < 5; i++) {

            double velocity2D = distanceVelocitiesTable.get(newDistance * 39.3701) * Math.cos(hoodAngle);

            double ydistance = Math.sin(newAngle) * newDistance;
            double xdistance = Math.cos(newAngle) * newDistance;

            double dx = dxdt * (newDistance / velocity2D);
            double dy = dydt * (newDistance / velocity2D);

            /*

            double EndpointX = x+xdistance+dx;
            double EndpointY = y+ydistance+dy;

            double newEndpointX = x+xdistance-dx;
            double newEndpointY = y+ydistance-dy;

            */

            double newDistanceX = xdistance - dx;
            double newDistanceY = ydistance - dy;

            newDistance = Math.hypot(newDistanceX, newDistanceY);
        }

        return newDistance;
    }

    public static final double RecurssiveFOTRotation(
            double x, double y, double dxdt, double dydt, double hoodAngle, double angle, double distance) {

        double newAngle = angle;
        double newDistance = distance;

        for (int i = 0; i < 5; i++) {

            double velocity2D = (distanceVelocitiesTable.get(newDistance * 39.3701) * Math.cos(hoodAngle));

            double ydistance = Math.sin(newAngle) * newDistance;
            double xdistance = Math.cos(newAngle) * newDistance;

            double dx = dxdt * (newDistance / velocity2D);
            double dy = dydt * (newDistance / velocity2D);

            /*

            double EndpointX = x+xdistance+dx;
            double EndpointY = y+ydistance+dy;

            double newEndpointX = x+xdistance-dx;
            double newEndpointY = y+ydistance-dy;

            */

            double newDistanceX = xdistance - dx;
            double newDistanceY = ydistance - dy;

            newAngle = Math.atan(newDistanceY / newDistanceX);
        }

        return newAngle;
    }
}
