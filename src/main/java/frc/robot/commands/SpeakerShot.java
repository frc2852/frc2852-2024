// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SpeakerShot extends SequentialCommandGroup {
  public SpeakerShot(Intake intakeSubsystem, Conveyor conveyorSubsystem, Shooter shooterSubsystem) {
    addCommands(
        new InstantCommand(() -> shooterSubsystem.resetState(), shooterSubsystem),

        // Get shooter rollers up to speed
        new RunCommand(() -> shooterSubsystem.flyWheelFullSpeed(), shooterSubsystem)
            .until(() -> shooterSubsystem.isShooterAtSpeed()),

        new WaitCommand(0.2),
        // Run intake, conveyor, shooter in parallel until the game piece is ready
        new ParallelCommandGroup(
            new RunCommand(() -> shooterSubsystem.flyWheelFullSpeed(), shooterSubsystem),
            new RunCommand(() -> intakeSubsystem.runIntake(true), intakeSubsystem),
            new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem))
            .until(() -> shooterSubsystem.hasGamePieceBeenShot()),

        // Stop intake, conveyor and shooter
        new ParallelCommandGroup(
            new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem),
            new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)));
  }
}
