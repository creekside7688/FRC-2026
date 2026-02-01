package frc.robot.subsystems.drivebase;

import java.util.Queue;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.drivebase.module.SparkOdometryThread;

public class GyroIONavX implements GyroIO {

    private final AHRS gyro;
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    public GyroIONavX(NavXComType port) {
        gyro = new AHRS(port);
        gyro.setAngleAdjustment(270);

        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(gyro::getAngle);

    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocityDegreesPerSec = gyro.getRate() * (DriveConstants.GYRO_INVERTED ? -1 : 1);

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

}
