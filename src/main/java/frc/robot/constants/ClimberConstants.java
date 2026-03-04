package frc.robot.constants;

public class ClimberConstants {
    public static final int LARGE_HOOK_MOTOR_ID = 9;
    //     public static final int SMALL_HOOK_MOTOR_ID = 10;
    //     public static final int GAP_BLOCKER_MOTOR_ID = 11;

    public static final double GEAR_RATIO = 125.0 / 1.0;

    public static final double CHAIN_RADIUS_INCHES = 1.685 / 2.0; // inches
    public static final double CHAIN_CIRCUMFERENCE_INCHES = 2 * Math.PI * CHAIN_RADIUS_INCHES; // inches
    public static final double CHAIN_CENTERS_DISTANCE_INCHES = 13.375; // inches
    public static final double CHAIN_LOOP_LENGTH_INCHES =
            CHAIN_CIRCUMFERENCE_INCHES + (2 * CHAIN_CENTERS_DISTANCE_INCHES); // inches
    public static final double POSITION_CONVERSION_FACTOR =
            CHAIN_CIRCUMFERENCE_INCHES / GEAR_RATIO; // inches per motor rotation

    //     public static final double HOOK_PULL_PERCENT = 0.85; // percent of chain down
    public static final double HOOK_RESET_PERCENT = 0.50; // percent of chain (back) down
    public static final double HOOK_POST_PERCENT = 0.35; // percent of chain down

    public static final double HOOK_RESET_TARGET_POSITION_INCHES = 0;
    //     CHAIN_CENTERS_DISTANCE_INCHES * HOOK_RESET_PERCENT; // inches
    //     3.5; // inches.

    public static final double HOOK_PRE_TARGET_POSITION_INCHES = -4.81;

    //     CHAIN_CIRCUMFERENCE_INCHES * 3 / 8; // 45 deg from hook point)
    public static final double HOOK_POST_TARGET_POSITION_INCHES = -13;
    //     CHAIN_CIRCUMFERENCE_INCHES * .5
    //     + HOOK_POST_PERCENT * CHAIN_CENTERS_DISTANCE_INCHES; // a little after horizontal level
    //     public static final double HOOK_PULL_TARGET_POSITION_INCHES =
    //     CHAIN_CIRCUMFERENCE_INCHES * .5 + HOOK_PULL_PERCENT * CHAIN_CENTERS_DISTANCE_INCHES; // pull down the chain

    public static final double HOOK_MOTOR_SPEED = .5; // percent output
    //     public static final double GAP_BLOCKER_MOTOR_SPEED = 0.5; // percent output

    public static final double HOOK_ACCURACY_INCHES = 0.5; // inches
    public static final double HOOK_FOCUS_ACCURACY_INCHES = 0.2; // inches
    //     public static final double GAP_BLOCKER_ACCURACY_ROTATIONS = 0.5; // rotations

    //     public static final boolean LARGE_HOOK_CLOCKWISE = true;
    //     public static final boolean SMALL_HOOK_CLOCKWISE = false;
    //     public static final boolean GAP_BLOCKER_CLOCKWISE = false;

    public static final double CLIMBER_TEST_SPEED = 1; // percent output

    public static final int STALL_CURRENT_LIMIT = 50;
    public static final int FREE_CURRENT_LIMIT = 30;
}
