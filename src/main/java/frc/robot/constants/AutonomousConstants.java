package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class AutonomousConstants {
    public static final double PATH_TRANSLATION_P = 5.0;
    public static final double PATH_TRANSLATION_I = 0;
    public static final double PATH_TRANSLATION_D = 0;

    public static final double PATH_THETA_P = 5;
    public static final double PATH_THETA_I = 0;
    public static final double PATH_THETA_D = 0;

    // Constraint for the motion profiled robot angle controller
    public static final PPHolonomicDriveController pfc = new PPHolonomicDriveController(
            new PIDConstants(PATH_TRANSLATION_P, PATH_TRANSLATION_I, PATH_THETA_D),
            new PIDConstants(PATH_THETA_P, PATH_THETA_I, PATH_THETA_D));

    public static enum PATHNAME {
        TOP_BLUE_TO_CENTER,
        TOP_CENTER_TO_BLUE,
        BOTTOM_BLUE_TO_CENTER,
        BOTTOM_CENTER_TO_BLUE,

        TOP_RED_TO_CENTER,
        TOP_CENTER_TO_RED,
        BOTTOM_RED_TO_CENTER,
        BOTTOM_CENTER_TO_RED,
    }

    public static String GET_PATH(PATHNAME path) {
        switch (path) {
            case TOP_BLUE_TO_CENTER:
                return "topbluetocenter";
            case TOP_CENTER_TO_BLUE:
                return "topcentertoblue";
            case BOTTOM_BLUE_TO_CENTER:
                return "bottombluetocenter";
            case BOTTOM_CENTER_TO_BLUE:
                return "bottomcentertoblue";

            case TOP_RED_TO_CENTER:
                return "topredtocenter";
            case TOP_CENTER_TO_RED:
                return "topcentertored";
            case BOTTOM_RED_TO_CENTER:
                return "bottomredtocenter";
            case BOTTOM_CENTER_TO_RED:
                return "bottomcentertored";
            default:
                return "";
        }
    }

    public static RobotConfig ROBOT_CONFIG;

    static {
        try {
            ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
