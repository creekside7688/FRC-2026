package frc.robot.subsystems.drivebase.module;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import static frc.lib.SparkUtils.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.ModuleConstants;

public class ModuleIOSparkMax implements ModuleIO {

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController turnPID;
  
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private final Rotation2d zeroRotation;

  public ModuleIOSparkMax(int driveMotorID, int turnMotorID, Rotation2d zeroRotation) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

    this.zeroRotation = zeroRotation;

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    drivePID = driveMotor.getClosedLoopController();
    turnPID = turnMotor.getClosedLoopController();

    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    turnConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    driveConfig.encoder
        .positionConversionFactor(ModuleConstants.DRIVE_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELOCITY_FACTOR);

    turnConfig.absoluteEncoder
        .positionConversionFactor(ModuleConstants.TURN_ENCODER_POSITION_FACTOR)
        .velocityConversionFactor(ModuleConstants.TURN_ENCODER_VELOCITY_FACTOR)
        .inverted(ModuleConstants.TURN_ENCODER_INVERTED);

    driveConfig.closedLoop
        .p(ModuleConstants.DRIVE_P)
        .i(ModuleConstants.DRIVE_I)
        .d(ModuleConstants.DRIVE_D)
        .velocityFF(ModuleConstants.DRIVE_FF)
        .outputRange(ModuleConstants.DRIVE_MINIMUM_OUTPUT, ModuleConstants.DRIVE_MAXIMUM_OUTPUT);

    turnConfig.closedLoop
        .p(ModuleConstants.TURN_P)
        .i(ModuleConstants.TURN_I)
        .d(ModuleConstants.TURN_D)
        .velocityFF(ModuleConstants.TURN_FF)
        .outputRange(ModuleConstants.TURN_MINIMUM_OUTPUT, ModuleConstants.TURN_MAXIMUM_OUTPUT)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(ModuleConstants.TURN_PID_MINIMUM_INPUT)
        .positionWrappingMaxInput(ModuleConstants.TURN_PID_MAXIMUM_INPUT);

    driveConfig
        .idleMode(ModuleConstants.DRIVE_IDLE_MODE)
        .smartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    turnConfig
        .idleMode(ModuleConstants.TURN_IDLE_MODE)
        .smartCurrentLimit(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder.setPosition(0);

    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(driveMotor, driveEncoder::getPosition);
    turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnMotor, turnEncoder::getPosition);
  }



  public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveMotor, driveEncoder::getPosition, (value) -> inputs.drivePositionMeters = value);
        ifOk(driveMotor, driveEncoder::getVelocity, (value) -> inputs.driveVelocityMetersPerSec = value);
        ifOk(
                driveMotor,
                new DoubleSupplier[] {driveMotor::getAppliedOutput, driveMotor::getBusVoltage},
                (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveMotor, driveMotor::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnMotor,
                turnEncoder::getPosition,
                (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnMotor, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnMotor,
                new DoubleSupplier[] {turnMotor::getAppliedOutput, turnMotor::getBusVoltage},
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnMotor, turnMotor::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
                timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsMeters =
                drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

  @Override
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  @Override
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  @Override
  public Rotation2d getTurnAngle() {
    return new Rotation2d(turnEncoder.getPosition()).minus(zeroRotation);
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSecond) {
    drivePID.setSetpoint(velocityMetersPerSecond, SparkMax.ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(Rotation2d angle) {
    turnPID.setSetpoint(angle.minus(zeroRotation).getRadians(), SparkMax.ControlType.kPosition);
  }

  @Override
  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }
}

