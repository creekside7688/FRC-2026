// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterLookup;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class testShootingLookup extends Command {
    /** Creates a new testVDS. */
    private final Shooter shooter;
    private final Feeder feeder;

    public testShootingLookup(Shooter shooter, Feeder feeder) {
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(shooter, feeder);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = shooter.getVariableDistance();
        double desiredRPM = ShooterLookup.lookupRPM(distance);
        double desiredAngle = ShooterLookup.lookupAngle(distance);
        shooter.SetRPM(desiredRPM);
        feeder.RunFeeder();
        // shooter.setHoodPosition(ShooterLookup.lookupAngle(desiredAngle));
        /*

        if (shooter.checkShooterRPMTolerance(desiredRPM) && shooter.checkShooterPositionTolerance(desiredAngle)) {
            shooter.RunFeeder();
        }
         */
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.RunIdle();
        feeder.stopFeeder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
