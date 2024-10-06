package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PrimeShooter extends Command {

  private final Shooter shooter;

  public PrimeShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.primeShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.isShooterReady();
  }
}
