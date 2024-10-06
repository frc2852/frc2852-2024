package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class PivotShootPosition extends Command {
  private final ShooterPivot shooterPivot;

  public PivotShootPosition(ShooterPivot shooterPivot) {
      this.shooterPivot = shooterPivot;
      addRequirements(this.shooterPivot);
  }

  @Override
  public void initialize() {
      shooterPivot.pivotShootPosition();
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
      return shooterPivot.isAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {

  }
}