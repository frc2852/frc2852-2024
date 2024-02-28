// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SpeakerShot extends SequentialCommandGroup {
  public SpeakerShot(Intake intakeSubsystem, Conveyor conveyorSubsystem, Shooter shooterSubsystem) {
    addCommands(
        // Get shooter rollers up to speed
        new RunCommand(() -> shooterSubsystem.flyWheelFullSpeed(), shooterSubsystem)
            .until(() -> shooterSubsystem.isShooterAtSpeed()),

        // Run intake, conveyor, shooter in parallel until the game piece is ready
        new ParallelCommandGroup(
            new RunCommand(() -> shooterSubsystem.flyWheelFullSpeed(), shooterSubsystem),
            new RunCommand(() -> intakeSubsystem.runIntake(true), intakeSubsystem),
            new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem))
            .withTimeout(2),

        // TODO: I'm worried that the game piece is going to pass by this sensor either too fast or too slow. If its too fast it will never be detected and the motors will never stop
        // If its detected too soon the motors will cut power well shooting affecting the shot.
        // So I've commented this out and added a timeout to the parallel command group above, this isn't the best fix but without having the robot to test its the best I can do for now.
        // .until(() -> shooterSubsystem.hasGamePieceBeenShot()),

        // Stop intake, conveyor and shooter
        new ParallelCommandGroup(
            new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem),
            new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem)));
  }
}
