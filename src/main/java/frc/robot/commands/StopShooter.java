package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class StopShooter extends InstantCommand {
  private final Shooter shooter;

  public StopShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    shooter.stopShooter();
  }
}