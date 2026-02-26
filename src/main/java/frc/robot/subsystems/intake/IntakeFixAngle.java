// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeFixAngle extends Command {
    Intake intake;
    double motorInput;

    public IntakeFixAngle(Intake inputIntake) {
        this.intake = inputIntake;
        this.addRequirements(new Subsystem[] {inputIntake});
    }

    public void initialize() {
        this.intake.setConstants();
        Timer.delay(0.5);
        this.intake.setIntakeAngle();
    }

    public void execute() {}

    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }
}
