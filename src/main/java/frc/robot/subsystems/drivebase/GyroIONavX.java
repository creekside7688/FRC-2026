package frc.robot.subsystems.drivebase;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivebase.module.SparkOdometryThread;

public class GyroIONavX implements GyroIO {

    private final AHRS gyro;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavX(NavXComType port) {
        gyro = new AHRS(port, (byte) DriveConstants.ODOMETRY_FREQUENCY);
        gyro.setAngleAdjustment(270);

        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> {
            return getRotation().getDegrees();
        });

    }

    @Override
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    @Override
    public double getAngularVelocity() {
        return gyro.getRate() * (DriveConstants.GYRO_INVERTED ? -1.0 : 1.0);
    }

    @Override
    public void reset() {
        gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPositionDegrees = getRotation();
        inputs.yawVelocityDegreesPerSec = getAngularVelocity();

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
