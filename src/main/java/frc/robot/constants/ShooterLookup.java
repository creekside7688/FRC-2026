package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            {60, 6.04623, 72.69}, // MINIMUM
        };

        final double[][] LookupTable = { // <= THIS ONE FOR RPMS!!!!
            {240, 4400, 59.86},
            {220, 4280, 60.45}, // MAXIMUM
            {200, 4160, 61.13},
            {180, 4020, 61.92},
            {160, 3920, 62.86},
            {140, 3790, 64},
            {120, 3570, 65.417},
            {100, 3400, 67.2},
            {80, 3200, 69.54},
            {60, 2980, 72.69}, // MINIMUM
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
        return distance <= 240 && distance >= 60;
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


    public static final Translation2d CalculationDelayOffset(
            Pose2d robotPose, double dxdt, double dydt, double calculationLatency) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation3d hubPose = (alliance == Alliance.Red) ? GameConstants.HUB_RED : GameConstants.HUB_BLUE;

        double x = robotPose.getX();
        double y = robotPose.getY();

        double distx = hubPose.getX() - x;
        double disty = hubPose.getY() - y;

        double dx = dxdt * calculationLatency;
        double dy = dydt * calculationLatency;

        if (distx == 0 && disty == 0) return new Translation2d(0, 0);

        double dAngle = (((dy * distx) - (dx * disty))) / (Math.pow((distx), 2) + Math.pow((disty), 2));

        double dDistance = (((distx * dx) + (disty * dy)) / (Math.sqrt(Math.pow(distx, 2) + Math.pow(disty, 2))));
        return new Translation2d(dDistance, dAngle); // in meters
    }

    // not done
    public static final Translation2d RecurssiveFOT(
            double dxdt, double dydt, double hoodAngle, double angle, double distance) {

        double newAngle = angle;
        double newDistance = distance;

        for (int i = 0; i < 5; i++) {

            double velocity2D = distanceVelocitiesTable.get(newDistance * 39.3701) * Math.cos(Math.toRadians(hoodAngle));

            double ydistance = Math.sin(Math.toRadians(newAngle)) * newDistance;
            double xdistance = Math.cos(Math.toRadians(newAngle)) * newDistance;

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

            newAngle = Math.toDegrees(Math.atan2(newDistanceY, newDistanceX));
            newDistance = Math.hypot(newDistanceX, newDistanceY);
        }

        //x value is distance, y is angle in degrees
        return new Translation2d(newDistance, (newAngle));
    }
}
