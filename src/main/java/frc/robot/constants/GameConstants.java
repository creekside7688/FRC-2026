package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class GameConstants {
    // Field Dimensions
    public static final Distance FIELD_LENGTH = Meters.of(16.513048);
    public static final Distance FIELD_WIDTH = Meters.of(8.042656);

    public static final Distance ALLIANCE_ZONE = Meters.of(3.963924);

    // Goal/Hub Positions
    public static final Translation2d HUB_RED = new Translation2d(Inches.of(469.11), Inches.of(158.84));
    public static final Translation2d HUB_BLUE = Translation2d.kZero;
    public static final Distance FUNNEL_RADIUS = Meters.of(0.6096);
    public static final Distance FUNNEL_HEIGHT = Meters.of(0.39624);

    // Trench and Bump Geometry
    public static final Distance TRENCH_BUMP_X = Meters.of(4.611624);
    public static final Distance TRENCH_WIDTH = Meters.of(1.266444);
    private static final Distance BUMP_INSET = TRENCH_WIDTH.plus(Meters.of(0.3048));
    private static final Distance BUMP_LENGTH = Meters.of(1.8542);

    private static final Distance TRENCH_ZONE_EXTENSION = Meters.of(1.778);
    private static final Distance BUMP_ZONE_EXTENSION = Meters.of(1.524);
    private static final Distance TRENCH_BUMP_ZONE_TRANSITION =
            TRENCH_WIDTH.plus(BUMP_INSET).div(2);

    public static final Translation2d[][] TRENCH_ZONES = {
        new Translation2d[] {
            new Translation2d(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION), Meters.of(0)),
            new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION)
        },
        new Translation2d[] {
            new Translation2d(
                    TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
            new Translation2d(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH)
        },
        new Translation2d[] {
            new Translation2d(FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)), Meters.of(0)),
            new Translation2d(
                    FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)), TRENCH_BUMP_ZONE_TRANSITION)
        },
        new Translation2d[] {
            new Translation2d(
                    FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(TRENCH_ZONE_EXTENSION)),
                    FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION)),
            new Translation2d(FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(TRENCH_ZONE_EXTENSION)), FIELD_WIDTH)
        }
    };

    public static final Translation2d[][] BUMP_ZONES = {
        new Translation2d[] {
            new Translation2d(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION), TRENCH_BUMP_ZONE_TRANSITION),
            new Translation2d(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), BUMP_INSET.plus(BUMP_LENGTH))
        },
        new Translation2d[] {
            new Translation2d(
                    TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION), FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
            new Translation2d(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION), FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
        },
        new Translation2d[] {
            new Translation2d(
                    FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)),
                    FIELD_WIDTH.minus(BUMP_INSET.plus(BUMP_LENGTH))),
            new Translation2d(
                    FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)),
                    FIELD_WIDTH.minus(TRENCH_BUMP_ZONE_TRANSITION))
        },
        new Translation2d[] {
            new Translation2d(FIELD_LENGTH.minus(TRENCH_BUMP_X.plus(BUMP_ZONE_EXTENSION)), TRENCH_BUMP_ZONE_TRANSITION),
            new Translation2d(
                    FIELD_LENGTH.minus(TRENCH_BUMP_X.minus(BUMP_ZONE_EXTENSION)), BUMP_INSET.plus(BUMP_LENGTH))
        }
    };

    public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
}
