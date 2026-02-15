package frc.robot.subsystems.drivebase.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.SparkUtils;
import frc.robot.constants.ModuleConstants;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOMapleSim implements ModuleIO {
    private static final double DRIVE_KP = 13.0;
    private static final double DRIVE_KI = 0.0;
    private static final double DRIVE_KD = 0.0;

    private static final double TURN_KP = 7.0;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.5;

    private boolean driveClosedLoop, turnClosedLoop;
    private double desiredVelocityMetersPerSec;
    private double desiredAngleRadians;

    private final SwerveModuleSimulation module;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private final PIDController driveController;
    private final PIDController turnController;

    private double driveAppliedVoltage, turnAppliedVoltage;

    public ModuleIOMapleSim(SwerveModuleSimulation module) {
        this.module = module;
        this.driveMotor = module.useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT));
        this.turnMotor = module.useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT));

        driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        turnController = new PIDController(TURN_KP, TURN_KI, TURN_KD);

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        updateSimulation();

        inputs.driveConnected = true;
        inputs.drivePositionMeters =
                module.getDriveWheelFinalPosition().in(Rotations) * ModuleConstants.WHEEL_CIRCUMFERENCE_METRES;
        inputs.driveVelocityMetersPerSec =
                module.getDriveWheelFinalSpeed().in(RotationsPerSecond) * ModuleConstants.WHEEL_CIRCUMFERENCE_METRES;
        inputs.driveAppliedVolts = module.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveCurrentAmps = Math.abs(module.getDriveMotorStatorCurrent().in(Amps));

        inputs.turnConnected = true;
        inputs.turnPositionRad = module.getSteerAbsoluteFacing();
        inputs.turnVelocityRadPerSec = module.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = module.getSteerMotorAppliedVoltage().in(Volts);
        inputs.turnCurrentAmps = Math.abs(module.getSteerMotorStatorCurrent().in(Amps));

        inputs.odometryTimestamps = SparkUtils.getSimulationOdometryTimeStamps();
        inputs.odometryDrivePositionsMeters = Arrays.stream(module.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Rotations) * ModuleConstants.WHEEL_CIRCUMFERENCE_METRES)
                .toArray();

        inputs.odometryTurnPositionsRad = module.getCachedSteerAbsolutePositions();
    }

    public void updateSimulation() {
        driveMotor.requestVoltage(module.config.driveMotorConfigs.calculateVoltage(
                Amps.of(0), RadiansPerSecond.of(desiredVelocityMetersPerSec / ModuleConstants.WHEEL_RADIUS_METRES)));

        turnMotor.requestVoltage(Volts.of(
                turnController.calculate(module.getSteerAbsoluteFacing().getRadians(), desiredAngleRadians)));
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond) {
        desiredVelocityMetersPerSec = velocityMetersPerSecond;
    }

    @Override
    public void setTurnPosition(Rotation2d angle) {
        desiredAngleRadians = angle.getRadians();
    }

    @Override
    public void resetDriveEncoder() {}

    @Override
    public void setDriveOpenLoop(double voltage) {
        driveClosedLoop = false;
        driveAppliedVoltage = voltage;
    }

    @Override
    public void setTurnOpenLoop(double voltage) {
        turnClosedLoop = false;
        turnAppliedVoltage = voltage;
    }
}
