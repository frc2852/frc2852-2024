package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class AutoShooterStart extends InstantCommand {
  public AutoShooterStart(Shooter shooterSubsystem) {
    super(() -> shooterSubsystem.autoShooterStart(), shooterSubsystem);
  }
}
