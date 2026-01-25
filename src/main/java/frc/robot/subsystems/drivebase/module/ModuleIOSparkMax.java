package frc.robot.subsystems.drivebase.module;

import frc.robot.constants.ModuleConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOSparkMax implements ModuleIO {

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SparkClosedLoopController drivePID;
  private final SparkClosedLoopController turnPID;

  public ModuleIOSparkMax(int driveMotorID, int turnMotorID) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

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
    return new Rotation2d(turnEncoder.getPosition());
  }

  @Override
  public void setDriveVelocity(double velocityMetersPerSecond) {
    drivePID.setSetpoint(velocityMetersPerSecond, SparkMax.ControlType.kVelocity);
  }

  @Override
  public void setTurnPosition(Rotation2d angle) {
    turnPID.setSetpoint(angle.getRadians(), SparkMax.ControlType.kPosition);
  }

  @Override
  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }


  @Override
  public void update() {
  }
}

