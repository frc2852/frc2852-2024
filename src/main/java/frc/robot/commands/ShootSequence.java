package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class ShootSequence extends SequentialCommandGroup {

  public ShootSequence(Shooter shooter, Intake intake) {
    addRequirements(shooter, intake);

    addCommands(
        new PrimeShooter(shooter),
        new MoveNoteToShooter(intake)
    );
  }
}
