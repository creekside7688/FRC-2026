<<<<<<< HEAD
package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * LimelightConstants
 */

public class VisionConstants {

    public static class TRANSLATE_CONTROLLER {
        public static final double kP = 1.9;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double TOLERANCE = 0.02;

        public static final double kV_MAX = 4.8;
        public static final double kA_MAX = 4.0;
    }

    public static class THETA_CONTROLLER {
        public static final double kP = 1.0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double TOLERANCE = Units.degreesToRadians(2.0);
        public static final double kV_MAX = 2 * Math.PI;
        public static final double kA_MAX = 5.0;
    }

//    public static final double GOAL_X = Units.inchesToMeters(15 + 3.5);
    public static final double GOAL_X = 1.0;
    public static final double GOAL_Y = 0;
    public static final double GOAL_Z = 0;

    public static final double ROBOT_CAMERA_OFFSET_FWD = Units.inchesToMeters(16);
    public static final double ROBOT_CAMERA_OFFSET_SIDE = -Units.inchesToMeters(10.5);
    public static final double ROBOT_CAMERA_OFFSET_VERT = 0;
    public static final double CAMERA_ROLL = 0;
    public static final double CAMERA_PITCH = 0;
    public static final double CAMERA_YAW = 0;

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(ROBOT_CAMERA_OFFSET_FWD,
            ROBOT_CAMERA_OFFSET_SIDE, ROBOT_CAMERA_OFFSET_VERT), new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW));

    public static final double HIGHEST_AMBIGUITY = 0.2;
    public static final String CAMERA_NAME = "7688Camera";

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
            new Rotation2d(Math.PI));
}
=======
package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * LimelightConstants
 */

public class VisionConstants {

    public static class TRANSLATE_CONTROLLER {
        public static final double kP = 2.0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double TOLERANCE = 0.01;

        public static final double kV_MAX = 0.8;
        public static final double kA_MAX = 0.05;
    }

    public static class THETA_CONTROLLER {
        public static final double kP = 1.0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double TOLERANCE = Units.degreesToRadians(10.0);
        public static final double kV_MAX = 2.0 * Math.PI;
        public static final double kA_MAX = 5.0;
    }

//    public static final double GOAL_X = Units.inchesToMeters(15 + 3.5);
    public static final double GOAL_X = Units.inchesToMeters(28);
    public static final double GOAL_Y = 0;
    public static final double GOAL_Z = 0;

    public static final double ROBOT_CAMERA_OFFSET_FWD = Units.inchesToMeters(16 + 3 + 3/8);
    public static final double ROBOT_CAMERA_OFFSET_SIDE = 0;
    public static final double ROBOT_CAMERA_OFFSET_VERT = Units.inchesToMeters(0.5);
    public static final double CAMERA_ROLL = 0;
    public static final double CAMERA_PITCH = 0;
    public static final double CAMERA_YAW = 0;

    public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(ROBOT_CAMERA_OFFSET_FWD,
            ROBOT_CAMERA_OFFSET_SIDE, ROBOT_CAMERA_OFFSET_VERT), new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, CAMERA_YAW));

    public static final double HIGHEST_AMBIGUITY = 0.2;
    public static final String CAMERA_NAME = "7688Camera";

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
            new Rotation2d(Math.PI));
}
>>>>>>> b21618d (working driveToPose + global robot positioning (marcus' basement edition))
