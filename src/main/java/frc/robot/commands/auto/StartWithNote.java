package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class StartWithNote extends InstantCommand {
  private final Intake intake;

  public StartWithNote(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.startWithNoteState();
  }
}