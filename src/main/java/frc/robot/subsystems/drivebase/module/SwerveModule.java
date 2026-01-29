package frc.robot.subsystems.drivebase.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  private final ModuleIO io;
  private final String descriptor;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  public SwerveModule(ModuleIO io, String descriptor) {
    this.io = io;
    this.descriptor = descriptor;
  }


  public void update() {
      io.updateInputs(inputs);
      Logger.processInputs("Drive/Module".concat(descriptor), inputs);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        io.getDriveVelocity(),
        io.getTurnAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        io.getDrivePosition(),
        io.getTurnAngle());
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  public void setDesiredState(SwerveModuleState state) {
    SwerveModuleState correctedState = new SwerveModuleState(
        state.speedMetersPerSecond,
        state.angle
    );

    correctedState.optimize(io.getTurnAngle());
    correctedState.cosineScale(io.getTurnAngle());

    io.setDriveVelocity(correctedState.speedMetersPerSecond);
    io.setTurnPosition(correctedState.angle);

    desiredState = state; 
  }

  public void resetEncoders() {
    io.resetDriveEncoder();
  }
}

