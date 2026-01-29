package frc.robot.subsystems.drivebase.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.ModuleConstants;

public class ModuleIOMapleSim implements ModuleIO {
    private static final double DRIVE_KP = 0.5;
    private static final double DRIVE_KI = 0.0;
    private static final double DRIVE_KD = 0.0;

    private static final double TURN_KP = 7.0;
    private static final double TURN_KI = 0.0;
    private static final double TURN_KD = 0.5;

    private final SwerveModuleSimulation module;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        0, 1 / (DCMotor.getNEO(1).withReduction(ModuleConstants.DRIVE_MOTOR_REDUCTION).KvRadPerSecPerVolt * ModuleConstants.WHEEL_RADIUS_METRES));

    private final PIDController driveController;
    private final PIDController turnController;

    private double driveAppliedVoltage, turnAppliedVoltage;

    private double desiredVelocityRadPerSec, desiredPositionRad;

    public ModuleIOMapleSim(SwerveModuleSimulation module) {
        this.module = module;
        this.driveMotor = module.useGenericMotorControllerForDrive()
                .withCurrentLimit(Units.Amps.of(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT));
        this.turnMotor = module.useGenericControllerForSteer()
                .withCurrentLimit(Units.Amps.of(ModuleConstants.TURN_MOTOR_CURRENT_LIMIT));


        driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        turnController = new PIDController(TURN_KP, TURN_KI, TURN_KD);

        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        updateSimulation();
        inputs.driveConnected = true;
        inputs.drivePositionMeters = getDrivePosition();
        inputs.driveVelocityMetersPerSec = getDriveVelocity();
        inputs.driveAppliedVolts = driveAppliedVoltage;
        inputs.driveCurrentAmps = Math.abs(module.getDriveMotorStatorCurrent().in(Amps));
    
        inputs.turnConnected = true;
        inputs.turnPosition = getTurnAngle();
        inputs.turnVelocityRadPerSec = module.getSteerAbsoluteEncoderSpeed().in(Units.RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVoltage;
        inputs.turnCurrentAmps = Math.abs(module.getSteerMotorStatorCurrent().in(Amps));

        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsMeters = new double[] {inputs.drivePositionMeters};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
    }

    public void updateSimulation() {
        driveAppliedVoltage = driveController.calculate(
            module.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond), desiredVelocityRadPerSec
        );

        turnAppliedVoltage = turnController.calculate(
            module.getSteerAbsoluteFacing().getRadians(), desiredPositionRad
        );

        driveMotor.requestVoltage(Units.Volts.of(driveAppliedVoltage));
        turnMotor.requestVoltage(Units.Volts.of(turnAppliedVoltage));
    }

    @Override
    public void setDriveVelocity(double velocityMetersPerSecond) {
        desiredVelocityRadPerSec = velocityMetersPerSecond / ModuleConstants.WHEEL_RADIUS_METRES;
    }

    @Override
    public void setTurnPosition(Rotation2d angle) {
        desiredPositionRad = angle.getRadians();
    }

    @Override
    public double getDriveVelocity() {
        return module.getDriveWheelFinalSpeed().in(RotationsPerSecond) * ModuleConstants.WHEEL_CIRCUMFERENCE_METRES;
    }

    @Override
    public double getDrivePosition() {
        return module.getDriveWheelFinalPosition().in(Units.Rotations) * ModuleConstants.WHEEL_CIRCUMFERENCE_METRES;
    }

    @Override
    public Rotation2d getTurnAngle() {
        return module.getSteerAbsoluteFacing();
    }

    @Override
    public void resetDriveEncoder() {
    }
}