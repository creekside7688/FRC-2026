package frc.robot.subsystems.drivebase.module;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

  @AutoLog
  public class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnPositionRad = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositionsRad = new Rotation2d[] {};
  }

  public void updateInputs(ModuleIOInputs inputs);

  public void setDriveVelocity(double velocityMetersPerSecond);

  public void setTurnPosition(Rotation2d angle);

  public void setDriveOpenLoop(double voltage);

  public void setTurnOpenLoop(double voltage);

  public void resetDriveEncoder();
}

