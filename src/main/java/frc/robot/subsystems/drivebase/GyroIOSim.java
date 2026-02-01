package frc.robot.subsystems.drivebase;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.SparkUtils;
import frc.robot.constants.DriveConstants;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyro;
    private double angleOffset = 0.0;

    public GyroIOSim(GyroSimulation gyro) {
        this.gyro = gyro;
        angleOffset = 90; //degrees
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyro.getGyroReading().plus(Rotation2d.fromDegrees(angleOffset));
        inputs.yawVelocityDegreesPerSec = gyro.getMeasuredAngularVelocity().in(DegreesPerSecond) * (DriveConstants.GYRO_INVERTED ? -1 : 1);

        inputs.odometryYawTimestamps = SparkUtils.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyro.getCachedGyroReadings();
    }

    @Override
    public void reset() {
        gyro.setRotation(Rotation2d.kZero);
    }
}
