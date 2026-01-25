package frc.robot.subsystems.drivebase.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveConstants;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyro;
    private double angleOffset = 0.0;

    public GyroIOSim(GyroSimulation gyro) {
        this.gyro = gyro;
        angleOffset = 90; //degrees
    }

    @Override
    public Rotation2d getRotation() {
        return gyro.getGyroReading().plus(Rotation2d.fromDegrees(angleOffset));
    }

    @Override
    // degrees per sec
    public double getAngularVelocity() {
        return gyro.getMeasuredAngularVelocity().magnitude() * (DriveConstants.GYRO_INVERTED ? -1 : 1) * 180 / Math.PI;
    }

    @Override
    public void reset() {
        gyro.setRotation(Rotation2d.kZero);
    }
}
