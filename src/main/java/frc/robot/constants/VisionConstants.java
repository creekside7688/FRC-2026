package frc.robot.constants;

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

    public static final String CAMERA_NAME_1 = "7688Camera";
    public static final String CAMERA_NAME_2 = "7688Camera";

    public static final Transform3d ROBOT_TO_CAM_TRANSFORM_1 = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    public static final Transform3d ROBOT_TO_CAM_TRANSFORM_2 = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

    public static final double HIGHEST_AMBIGUITY = 0.3;

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    public static final Pose2d FLIPPING_POSE = new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
            new Rotation2d(Math.PI));
}
