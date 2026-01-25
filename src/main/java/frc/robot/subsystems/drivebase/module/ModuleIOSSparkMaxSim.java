package frc.robot.subsystems.drivebase.module;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ModuleConstants;

public class ModuleIOSSparkMaxSim implements ModuleIO {

    private final SparkMaxSim driveSim;
    private final SparkMaxSim turnSim;
    private final SparkMax drive;
    private final SparkMax turn;

    private final SparkRelativeEncoderSim driveEncoder;
    private final SparkAbsoluteEncoderSim turnEncoder;

    private final DCMotor driveGearbox = DCMotor.getNEO(1);
    private final DCMotor turnGearbox = DCMotor.getNeo550(1);

    private final DCMotorSim driveIO = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getNEO(1), 0.001, ModuleConstants.DRIVE_MOTOR_REDUCTION),
            DCMotor.getNEO(1));

    private final DCMotorSim turnIO = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getNeo550(1), 0.001, ModuleConstants.TURN_MOTOR_REDUCTION),
            DCMotor.getNeo550(1));

    private static final double LOOP_PERIOD_SECS = 0.02;

    private double driveSetpoint = 0.0;
    private double turnSetpoint = 0.0;

    public ModuleIOSSparkMaxSim(int driveid, int turnid) {

        drive = new SparkMax(driveid, MotorType.kBrushless);
        turn = new SparkMax(turnid, MotorType.kBrushless);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();

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

        drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turn.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveSim = new SparkMaxSim(drive, driveGearbox);
        turnSim = new SparkMaxSim(turn, turnGearbox);


        driveEncoder = driveSim.getRelativeEncoderSim();
        turnEncoder = turnSim.getAbsoluteEncoderSim();
    }

    @Override
    public void update() {

        double driveAppliedVoltage = driveSim.getAppliedOutput() * 12;

        driveIO.setInputVoltage(driveAppliedVoltage);
        driveIO.update(LOOP_PERIOD_SECS);

        double driveEstimatedVel = driveIO.getAngularVelocityRPM() * ModuleConstants.WHEEL_CIRCUMFERENCE_METRES / 60;
        driveSim.iterate(driveEstimatedVel, 12.0, LOOP_PERIOD_SECS);

        double turnAppliedVoltage = turn.getAppliedOutput() * 12;

        turnIO.setInputVoltage(turnAppliedVoltage);
        turnIO.update(LOOP_PERIOD_SECS);

        double turnEstimatedVel = turnIO.getAngularVelocityRadPerSec();
        turnSim.iterate(turnEstimatedVel, 12.0, LOOP_PERIOD_SECS);

        System.out.println(driveEstimatedVel);
        System.out.println(turnEstimatedVel);
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
        driveSetpoint = velocityMetersPerSecond;
        drive.getClosedLoopController().setReference(velocityMetersPerSecond, ControlType.kVelocity);
    }

    @Override
    public void setTurnPosition(Rotation2d angle) {
        turnSetpoint = angle.getRadians();
        turn.getClosedLoopController().setReference(angle.getRadians(), ControlType.kPosition);
    }

    @Override
    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }
}