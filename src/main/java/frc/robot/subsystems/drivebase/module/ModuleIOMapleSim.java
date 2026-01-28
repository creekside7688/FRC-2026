package frc.robot.subsystems.drivebase.module;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
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
    private void calculateDriveControlLoop() {
        driveAppliedVoltage = driveController.calculate(
            module.getDriveWheelFinalSpeed().in(Units.RadiansPerSecond), desiredVelocityRadPerSec
        );
    }

    private void calculateTurnControlLoop() {
        turnAppliedVoltage = turnController.calculate(
            module.getSteerAbsoluteFacing().getRadians(), desiredPositionRad
        );
    }

    @Override
    public void update() {
        calculateDriveControlLoop();
        calculateTurnControlLoop();

        // System.out.println(driveMotor.getAppliedVoltage() + "||" + driveAppliedVoltage);

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
        return module.getCurrentState().speedMetersPerSecond;
    }

    @Override
    public double getDrivePosition() {
        return module.getDriveWheelFinalPosition().in(Units.Rotations) * ModuleConstants.WHEEL_CIRCUMFERENCE_METRES;
    }

    @Override
    public Rotation2d getTurnAngle() {
        return module.getCurrentState().angle;
    }

    @Override
    public void resetDriveEncoder() {
    }
}