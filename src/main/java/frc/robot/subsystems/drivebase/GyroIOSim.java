package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.SparkUtils;
import frc.robot.constants.DrivebaseConstants;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyro;
    private double angleOffset = 0.0;

    public GyroIOSim(GyroSimulation gyro) {
        this.gyro = gyro;
        angleOffset = 00; // degrees
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyro.getGyroReading().plus(Rotation2d.fromDegrees(angleOffset));
        inputs.yawVelocityDegreesPerSec =
                gyro.getMeasuredAngularVelocity().in(DegreesPerSecond) * (DrivebaseConstants.GYRO_INVERTED ? -1 : 1);

        inputs.odometryYawTimestamps = SparkUtils.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyro.getCachedGyroReadings();
    }

    @Override
    public void reset() {
        gyro.setRotation(Rotation2d.kZero);
    }
}
