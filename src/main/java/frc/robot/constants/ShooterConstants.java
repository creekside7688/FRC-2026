package frc.robot.constants;

public class ShooterConstants {
    public static final int BALL_SHOOTING_MOTOR_ID1 = 11; // "leader" motor
    public static final int BALL_SHOOTING_MOTOR_ID2 = 12; // all placeholder values
    public static final int BALL_HOOD_MOTOR = 20;
    public static final int FEED_MOTOR_SRX_ID = 9;

    public static final double SHOOTER_KV = 0.0021101;
    public static final double SHOOTER_KA = 0.00013519;
    public static final double SHOOTER_KS = 0.12074;

    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;

    public static final int IDLE_RPM = 500;
    public static final int SHOOTING_RPM = 2000;

    public static final double RUN_FEEDER_OUTPUT = -1.0;

    public static final double ANGLECHANGE_PER_ROTATION = 360.0 / 1300.0;
}
