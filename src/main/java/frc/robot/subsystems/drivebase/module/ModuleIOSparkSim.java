// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drivebase.module;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.ModuleConstants;

/** Physics sim implementation of module IO. */
public class ModuleIOSparkSim extends ModuleIOSparkMax {
  private final SparkMaxSim driveSim;
  private final SparkMaxSim turnSim;

  private final DCMotorSim driveGearboxSim;
  private final DCMotorSim turnGearboxSim;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final SwerveModuleSimulation moduleSimulation;

  public ModuleIOSparkSim(int driveID, int turnID, SwerveModuleSimulation moduleSimulation) {
    // Create drive and turn sim models
    super(driveID, turnID, Rotation2d.kZero);
    driveSim = new SparkMaxSim(driveMotor, DCMotor.getNEO(1));
    turnSim = new SparkMaxSim(turnMotor, DCMotor.getNeo550(1));
 
  driveGearboxSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, ModuleConstants.DRIVE_MOTOR_REDUCTION),
              DCMotor.getNEO(1));
  turnGearboxSim =
          new DCMotorSim(
              LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.004, ModuleConstants.TURN_MOTOR_REDUCTION),
              DCMotor.getNeo550(1));

    this.moduleSimulation = moduleSimulation;
    this.moduleSimulation.useDriveMotorController(new SparkSimControllerForDrive());
    this.moduleSimulation.useSteerMotorController(new SparkSimControllerForSteer());
  }

  public class SparkSimControllerForDrive implements SimulatedMotorController {

    private double requestedVoltage;
    @Override
    public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle,
        AngularVelocity encoderVelocity) {
          return Voltage.ofBaseUnits(requestedVoltage, Volts);
    }

    public void requestVoltage(double volts) {
      requestedVoltage = volts;
    }

  }
  
  public class SparkSimControllerForSteer implements SimulatedMotorController {

    private double requestedVoltage;

    @Override
    public Voltage updateControlSignal(Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle,
        AngularVelocity encoderVelocity) {
          return Voltage.ofBaseUnits(turnAppliedVolts, Volts);
    }

    public void requestVoltage(double volts) {
      requestedVoltage = volts;
    }

  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {


    driveAppliedVolts = driveSim.getAppliedOutput() * SimulatedBattery.getBatteryVoltage().magnitude();
    driveGearboxSim.setInput(driveAppliedVolts);

    turnAppliedVolts = turnSim.getAppliedOutput() * SimulatedBattery.getBatteryVoltage().magnitude();
    turnGearboxSim.setInput(turnAppliedVolts);

    driveGearboxSim.update(0.02);
    turnGearboxSim.update(0.02);

    driveSim.iterate(driveGearboxSim.getAngularVelocityRadPerSec() * ModuleConstants.WHEEL_RADIUS_METRES, SimulatedBattery.getBatteryVoltage().magnitude(), 0.02);
    turnSim.iterate(turnGearboxSim.getAngularVelocityRadPerSec(), SimulatedBattery.getBatteryVoltage().magnitude(), 0.02);

    super.updateInputs(inputs);
  }

  @Override
  public void resetDriveEncoder() {

  }
}