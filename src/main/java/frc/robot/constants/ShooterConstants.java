package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterConstants {
    public static final int BALL_SHOOTING_MOTOR_ID1 = 18; // "leader" motor
    public static final int BALL_SHOOTING_MOTOR_ID2 = 12;
    public static final int BALL_HOOD_MOTOR = 9;

    public static final double SHOOTER_KV = 0.0021152;
    public static final double SHOOTER_KA = 0.00024505;
    public static final double SHOOTER_KS = 0.12126;

    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;

    public static final int IDLE_RPM = 500;
    public static final int SHOOTING_RPM = 2000;

    public static final double RUN_FEEDER_OUTPUT = -0.9;

    public static final double INDEXER_VOLTAGE = 5;

    public static final Transform2d ROBOT_TO_SHOOTER =
            new Transform2d(new Translation2d(Inches.of(-5), Inches.of(-8)), Rotation2d.kZero);

    public static final int STALL_CURRENT_LIMIT = 50;
    public static final int FREE_CURRENT_LIMIT = 40;
}
