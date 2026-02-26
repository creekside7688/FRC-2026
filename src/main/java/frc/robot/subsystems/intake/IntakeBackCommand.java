// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeBackCommand extends Command {
   Intake intake;
   double motorInput;

   public IntakeBackCommand(Intake inputIntake) {
      this.intake = inputIntake;
      this.addRequirements(new Subsystem[]{inputIntake});
   }

   public void initialize() {
   }

   public void execute() {
      this.intake.setSpeedIntake(-0.2);
   }

   public void end(boolean interrupted) {
      this.intake.setSpeedIntake(0.0);
   }

   public boolean isFinished() {
      return false;
   }
}
