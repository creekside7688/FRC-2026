package frc.robot.constants;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * LimelightConstants
 */

public class VisionConstants {

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final String CAMERA_NAME_1 = "SWERVE_CAM";
    public static final String CAMERA_NAME_2 = "HOPPER_CAM";

    public static final Transform3d ROBOT_TO_SWERVE_CAM_TRANSFORM = new Transform3d(
        Inches.of(12.25345),
        Inches.of(-13.3305), 
        Inches.of(8.54079), 
        new Rotation3d(Degrees.of(0), 
            Degrees.of(15),
            Degrees.of(180)));

    public static final Transform3d ROBOT_TO_HOPPER_CAM_TRANSFORM = new Transform3d(
        Inches.of(-7.62845), 
        Inches.of(-7.09597), 
        Inches.of(20.37956), 
        new Rotation3d(
            Degrees.of(0),
            Degrees.of(15), 
            Degrees.of(0)));

    public static final double HIGHEST_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR = 0.75;

    public static double LINEAR_STDDEV_BASE = 0.02; //Meters
    public static double ANGULAR_STDDEV_BASE = 0.06; //Radians

    public static double[]  CAM_STD_DEVS = new double[] {
        1.0, //CAM 1
        1.0 //CAM 2
    };

    public static final double LINEAR_STD_DEV_MEGATAG2FACTOR = 0.5;
    public static final double ANGULAR_STD_DEV_MEGATAG2FACTOR = Double.POSITIVE_INFINITY;

    public static class OV9281 {
        public static final double HORIZONTAL_FOV_DEG = 70; 
        public static final double VERTICAL_FOV_DEG = 47.3;
        public static final double DIAGONAL_FOV_DEG = 79.1;

    }
}
