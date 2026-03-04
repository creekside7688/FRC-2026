package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class AutonomousConstants {
    public static final double PATH_TRANSLATION_P = 5.0;
    public static final double PATH_TRANSLATION_I = 0;
    public static final double PATH_TRANSLATION_D = 0;

    public static final double PATH_THETA_P = 5;
    public static final double PATH_THETA_I = 0;
    public static final double PATH_THETA_D = 0;

    // Constraint for the motion profiled robot angle controller
    public static final PPHolonomicDriveController PATHFINDING_CONTROLLER = new PPHolonomicDriveController(
            new PIDConstants(PATH_TRANSLATION_P, PATH_TRANSLATION_I, PATH_THETA_D),
            new PIDConstants(PATH_THETA_P, PATH_THETA_I, PATH_THETA_D));

    public static RobotConfig ROBOT_CONFIG;

    static {
        try {
            ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static final Pose2d LEFT_RED_CLIMB = new Pose2d(new Translation2d(15.024, 3.9), Rotation2d.k180deg);
    public static final Pose2d RIGHT_RED_CLIMB =
            new Pose2d(new Translation2d(15.024, 3.909 + Units.inchesToMeters(32.25 + 1.5 * 1.5)), Rotation2d.k180deg);
}
