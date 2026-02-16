package frc.robot.subsystems.drivebase.module;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    public class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public Rotation2d turnPositionRad = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsMeters = new double[] {};
        public Rotation2d[] odometryTurnPositionsRad = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveVelocity(double velocityMetersPerSecond) {}

    public default void setTurnPosition(Rotation2d angle) {}

    public default void setDriveOpenLoop(double voltage) {}

    public default void setTurnOpenLoop(double voltage) {}

    public default void resetDriveEncoder() {}
}
