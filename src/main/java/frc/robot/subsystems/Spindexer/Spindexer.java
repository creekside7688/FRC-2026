// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Spindexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    /** Creates a new Spindexer. */
    private final SparkMax indexerMotor = new SparkMax(SpindexerConstants.BALL_INDEXER_MOTOR_ID, MotorType.kBrushless);

    public Spindexer() {}

    public void setIndexer(double IndexerVoltage) {
        indexerMotor.setVoltage(IndexerVoltage);
    }

    public void runIndexer() {
        indexerMotor.setVoltage(SpindexerConstants.SPINDEXER_VOLTAGE);
    }

    public void stopIndexer() {
        indexerMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
