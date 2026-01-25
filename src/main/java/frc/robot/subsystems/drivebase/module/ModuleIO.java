package frc.robot.subsystems.drivebase.module;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

  public class  IOInputs {
  }

  public double getDriveVelocity();

  public double getDrivePosition();

  public Rotation2d getTurnAngle();

  public void setDriveVelocity(double velocityMetersPerSecond);

  public void setTurnPosition(Rotation2d angle);

  public void resetDriveEncoder();

  public void update();
}

