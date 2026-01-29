package frc.robot.subsystems.drivebase;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    @AutoLog
    public class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPositionDegrees = Rotation2d.kZero;
        public double yawVelocityDegreesPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public Rotation2d getRotation();

    public double getAngularVelocity();

    public void reset();

    public void updateInputs(GyroIOInputs inputs);
}
