package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class ReverseIntake extends Command {

  private final Intake intake;

  public ReverseIntake(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.reverse();
  }

  @Override
  public void execute() {
    // Continue reversing if needed
  }

  @Override
  public void end(boolean interrupted) {
    intake.resetState();
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}