// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeRollerForwardCommand extends Command {
    Intake intake;
    double motorInput;

    public IntakeRollerForwardCommand(Intake inputIntake) {
        this.intake = inputIntake;
        this.addRequirements(new Subsystem[] {inputIntake});
    }

    public void initialize() {}

    public void execute() {
        this.intake.setSpeedIntakeRoller(0.1);
    }

    public void end(boolean interrupted) {
        this.intake.stopIntakeRoller();
    }

    public boolean isFinished() {
        return false;
    }
}
