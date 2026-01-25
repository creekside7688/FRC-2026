package frc.robot.subsystems.drivebase.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.DriveConstants;

public class GyroIONavX implements GyroIO {

    private final AHRS gyro;

    public GyroIONavX(NavXComType port) {
        gyro = new AHRS(port);
        gyro.setAngleAdjustment(270);

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
}
