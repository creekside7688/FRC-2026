// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAgitate extends Command {
  /** Creates a new IntakeAgitate. */
  Intake intake;
  double previousTime;
  int direction = 0;
  
  public IntakeAgitate(Intake input) {
    this.intake = input;
    this.addRequirements(new Subsystem[] {input});
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    direction = 1;
    previousTime = Timer.getTimestamp();
    intake.setSpeedIntakeRoller(-0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(Timer.getTimestamp() - previousTime > 0.1)
    {
      previousTime = Timer.getTimestamp();
      direction = direction * -1;
      intake.setSpeedIntake(direction * 0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeRoller();
    intake.setIntakeAngleBack();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
