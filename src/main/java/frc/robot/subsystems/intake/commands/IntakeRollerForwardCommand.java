// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LEDLights;

public class IntakeRollerForwardCommand extends Command {
    Intake intake;
    LEDLights lights;
    double motorInput;
    InstantCommand flashLED;

    public IntakeRollerForwardCommand(Intake inputIntake, LEDLights inputLights) {
        this.intake = inputIntake;
        this.lights = inputLights;
        this.addRequirements(new Subsystem[] {inputIntake, inputLights});
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
