// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AmpNoteDischarge extends SequentialCommandGroup {

  public AmpNoteDischarge(Intake intakeSubsystem, Conveyor conveyorSubsystem, Shooter shooterSubsystem, Elevator elevatorSubsystem) {
    addCommands(
        // Run intake, conveyor, shooter in parallel until the game piece is ready
        new ParallelCommandGroup(
            new RunCommand(() -> intakeSubsystem.runIntake(true), intakeSubsystem),
            new RunCommand(() -> conveyorSubsystem.runConveyorForwardAmp(), conveyorSubsystem),
            new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem))
            .until(() -> conveyorSubsystem.isGamePieceAmpReady()),

        // Stop intake, conveyor and shooter
        new ParallelCommandGroup(
            new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem),
            new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)),

        // Then, move the elevator to amp position
        new RunCommand(() -> elevatorSubsystem.ampPosition(), elevatorSubsystem)
            .until(() -> elevatorSubsystem.isElevatorAtPosition()),

        // Run the conveyor and shooter again to discharge the game piece
        new ParallelCommandGroup(
            new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem),
            new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem))
            .until(() -> !conveyorSubsystem.isGamePieceAmpReady()),

        // Finally, stop the conveyor and shooter
        new ParallelCommandGroup(
            new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)),

        // Then, move the elevator to drive position
        new RunCommand(() -> elevatorSubsystem.drivePosition(), elevatorSubsystem)
            .until(() -> elevatorSubsystem.isElevatorAtPosition()));
  }
}