// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterFeeder;
import frc.robot.subsystems.shooter.ShooterHood;
import frc.robot.subsystems.spindexer.Spindexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestVDS extends Command {
    /** Creates a new testLookup. */
    private final Shooter shooter;

    private final ShooterHood shooterhood;

    private final ShooterFeeder feeder;

    private final Spindexer spindexer;

    public TestVDS(Shooter shooter, ShooterHood shooterhood, ShooterFeeder feeder, Spindexer spindexer) {
        this.shooter = shooter;
        this.shooterhood = shooterhood;
        this.feeder = feeder;
        this.spindexer = spindexer;
        addRequirements(shooter, shooterhood, feeder, spindexer);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.SetVariableRPM();
        shooterhood.setVariableHoodPosition();
        if (shooter.checkVariableRPMTolerance() && shooterhood.checkVariablePositionTolerance()) {
            feeder.RunFeeder();
            spindexer.runIndexer();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterMotor1Voltage(0);
        feeder.stopFeeder();
        spindexer.stopIndexer();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
