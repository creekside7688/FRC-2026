package frc.robot.subsystems.drivebase.module;

import java.util.concurrent.atomic.AtomicLong;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class SwerveModule {

  private final ModuleIO io;
  private final String descriptor;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private final Alert driveDisconnected;
  private final Alert turnDisconnected;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(ModuleIO io, String descriptor) {
    this.io = io;
    this.descriptor = descriptor;

    driveDisconnected =
                  new Alert("Disconnected drive motor on " + descriptor + " module.", AlertType.kError);
    turnDisconnected =
                  new Alert("Disconnected turn motor on " + descriptor + " module.", AlertType.kError);
  }

  public void update() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + descriptor, inputs);

    int numSamples = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[numSamples];
    for (int i = 0; i < numSamples; i++) {
      double positionMeters = inputs.odometryDrivePositionsMeters[i];
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        inputs.driveVelocityMetersPerSec,
        inputs.turnPosition);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        inputs.drivePositionMeters,
        inputs.turnPosition);
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState correctedState = new SwerveModuleState(
        state.speedMetersPerSecond,
        state.angle);

    correctedState.optimize(io.getTurnAngle());
    correctedState.cosineScale(io.getTurnAngle());

    io.setDriveVelocity(correctedState.speedMetersPerSecond);
    io.setTurnPosition(correctedState.angle);

    desiredState = state;
  }

  public void resetEncoders() {
    io.resetDriveEncoder();
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

}
