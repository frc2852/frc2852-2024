package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class MoveNoteToShooter extends InstantCommand {
  private final Intake intake;

  public MoveNoteToShooter(Intake intake) {
    this.intake = intake;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    intake.moveNoteToShooter();
  }
}