package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class StopShooterSequence extends SequentialCommandGroup {

    public StopShooterSequence(Intake intake, Shooter shooter, ShooterPivot shooterPivot) {
        addRequirements(shooter, shooterPivot);

        addCommands(
                new IntakeSeeking(intake),
                new StopShooter(shooter),
                new PivotLoadPosition(shooterPivot));
    }
}
