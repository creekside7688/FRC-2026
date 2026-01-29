package frc.lib;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ModuleConstants;

public final class SimUtils {

    public static class SimulatedDriveSparkMaxController implements SimulatedMotorController {
        private final SparkMaxSim sparkMaxSim;
        private final SparkMax max;

        private final SparkRelativeEncoderSim encoder;

        private final DoubleSupplier driveVelocitySupplier;
        private final DoubleSupplier drivePositionSupplier;
        
        private double desiredVoltage;

        public SimulatedDriveSparkMaxController(SparkMax max, DoubleSupplier driveVelocitySupplier, DoubleSupplier drivePositionSupplier) {
            this.max = max;
            

            sparkMaxSim = new SparkMaxSim(this.max, DCMotor.getNEO(1));

            this.driveVelocitySupplier = driveVelocitySupplier;
            this.drivePositionSupplier = drivePositionSupplier;

            this.encoder = sparkMaxSim.getRelativeEncoderSim();

        }

        @Override
        public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle,
                AngularVelocity encoderVelocity) {
                    encoder.setPosition(drivePositionSupplier.getAsDouble());
                    sparkMaxSim.iterate(driveVelocitySupplier.getAsDouble(), SimulatedBattery.getBatteryVoltage().magnitude(), SimulatedArena.getSimulationDt().in(Units.Seconds));
                    return Volts.of(sparkMaxSim.getAppliedOutput() * sparkMaxSim.getBusVoltage());
                }            

        public void requestVelocity(double velocityMetersPerSecond) {
            max.getClosedLoopController().setSetpoint(velocityMetersPerSecond, ControlType.kVelocity);
        }

        public void requestPosition(double radians) {
            max.getClosedLoopController().setSetpoint(radians, ControlType.kPosition);
        }
    }
    public static class SimulatedTurnSparkMaxController implements SimulatedMotorController {
        public final SparkMaxSim sparkMaxSim;
        private final SparkMax max;

        private final DoubleSupplier turnVelocitySupplier;
        private final Supplier<Rotation2d> turnPositionSupplier;

        private final SparkAbsoluteEncoderSim absoluteEncoder;
        public SimulatedTurnSparkMaxController(SparkMax max, DoubleSupplier turnVelocitySupplier, Supplier<Rotation2d> turnPositionSupplier) {
            this.max = max;

            sparkMaxSim = new SparkMaxSim(this.max, DCMotor.getNeo550(1));

            this.turnVelocitySupplier = turnVelocitySupplier;
            this.turnPositionSupplier = turnPositionSupplier;

            this.absoluteEncoder = sparkMaxSim.getAbsoluteEncoderSim();
        }

        @Override
        public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle,
                AngularVelocity encoderVelocity) {
                    this.absoluteEncoder.setPosition(turnPositionSupplier.get().getRadians());
                    sparkMaxSim.iterate(turnVelocitySupplier.getAsDouble(), SimulatedBattery.getBatteryVoltage().magnitude(), SimulatedArena.getSimulationDt().in(Units.Seconds));                   
                    return Volts.of(sparkMaxSim.getAppliedOutput() * sparkMaxSim.getBusVoltage());
                }            

        public void requestVelocity(double velocityMetersPerSecond) {
            max.getClosedLoopController().setSetpoint(velocityMetersPerSecond, ControlType.kVelocity);
        }

        public void requestPosition(double radians) {
            max.getClosedLoopController().setSetpoint(radians, ControlType.kPosition);
        }
    }
}
