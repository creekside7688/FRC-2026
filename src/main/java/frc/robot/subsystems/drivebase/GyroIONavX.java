package frc.robot.subsystems.drivebase;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DrivebaseConstants;
import frc.robot.subsystems.drivebase.module.SparkOdometryThread;
import java.util.Queue;

public class GyroIONavX implements GyroIO {

    private final AHRS gyro;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavX() {
        gyro = new AHRS(DrivebaseConstants.GYRO_PORT, (int) DrivebaseConstants.ODOMETRY_FREQUENCY);
        // gyro.configureVelocity(false, false, false, false);

        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(gyro::getAngle);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-gyro.getAngle());
        inputs.yawVelocityDegreesPerSec = -gyro.getRawGyroZ();

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double rot) -> Rotation2d.fromDegrees(-rot))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void reset() {
        gyro.reset();
    }
}
