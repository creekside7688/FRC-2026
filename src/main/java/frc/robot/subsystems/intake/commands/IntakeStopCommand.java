// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeStopCommand extends Command {
    /** Creates a new IntakeStopCommand. */
    Intake intake;

    double motorInput;

    public IntakeStopCommand(Intake inputIntake) {
        this.intake = inputIntake;
        this.addRequirements(new Subsystem[] {inputIntake});
    }

    public void initialize() {}

    public void execute() {
        this.intake.setSpeedIntakeRoller(-0.6);
    }

    public void end(boolean interrupted) {
        this.intake.stopIntakeRoller();
    }

    public boolean isFinished() {
        return false;
    }
}
