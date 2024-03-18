package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class AutoShooterStop extends InstantCommand {
  public AutoShooterStop(Shooter shooterSubsystem) {
    super(() -> shooterSubsystem.autoShooterStop(), shooterSubsystem);
  }
}
