package frc.robot.constants;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 15;
    public static final double INTAKE_PID_P = 1.5;
    public static final double INTAKE_PID_I = 0;
    public static final double INTAKE_PID_D = 0;
    public static final double INTAKE_FEEDFORWARD_S = 0;
    public static final double INTAKE_FEEDFORWARD_V = 0;
    public static final double INTAKE_FEEDFORWARD_A = 0;
    public static final double INTAKE_FEEDFORWARD_G = 0;
    public static final double INTAKE_SOFT_FORWARD_LIMIT = 30.0;
    public static final double INTAKE_SOFT_REVERSED_LIMIT = 0;
    public static final int INTAKE_ROLLER_MOTOR_ID = 14;
    public static final double POSITION_CONVERSION_FACTOR = (45.0 / 336.0);

    public static final double INTAKE_SPEED = 0.2;
    public static final double INTAKE_ROLLER_SPEED = 0.1;

    public static final int PIVOT_STALL_CURRENT_LIMIT = 70;
    public static final int PIVOT_FREE_CURRENT_LIMIT = 60;

    public static final int ROLLER_STALL_CURRENT_LIMIT = 50;
    public static final int ROLLER_FREE_CURRENT_LIMIT = 40;
}
