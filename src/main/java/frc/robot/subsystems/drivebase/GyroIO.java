package frc.robot.subsystems.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    public class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityDegreesPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public void reset();

    public void updateInputs(GyroIOInputs inputs);
}
