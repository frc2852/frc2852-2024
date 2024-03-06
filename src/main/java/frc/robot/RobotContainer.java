package frc.robot;

import frc.robot.Constants.OperatorConstant;
import frc.robot.commands.AmpNoteDischarge;
import frc.robot.commands.SpeakerShot;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.ClimbWheel;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Winch;
import frc.robot.util.DataTracker;
import frc.robot.util.constants.LogConstants;
import frc.robot.util.swerve.SwerveUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * RobotContainer is the class where the bulk of the robot's systems are
 * declared.
 * Here, subsystems, OI devices, and commands are set up and should be the only
 * place that this configuration exists in the code.
 */
public class RobotContainer {

  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  private SendableChooser<Command> autoChooser;

  private final Drive driveSubsystem;
  private final Conveyor conveyorSubsystem;
  private final Elevator elevatorSubsystem;
  private final Intake intakeSubsystem;
  private final Shooter shooterSubsystem;
  private final Winch winchSubsystem;
  private final ClimbWheel climbWheelSubsystem;

  /**
   * Constructs the container for the robot. Subsystems and command mappings are
   * initialized here.
   */
  public RobotContainer() {

    // Start data logger
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Port forwarding
    PortForwarder.add(5800, "photonvision.local", 5800);

    // Initialize data tracker
    DataTracker.putBoolean(LogConstants.ROBOT_SYSTEM, "Intialization", true, false);

    // Initialize controllers with distinct ports
    driverController = new CommandXboxController(OperatorConstant.DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OperatorConstant.OPERATOR_CONTROLLER_PORT);

    // Initialize subsystems
    driveSubsystem = new Drive();
    conveyorSubsystem = new Conveyor();
    elevatorSubsystem = new Elevator();
    intakeSubsystem = new Intake();
    shooterSubsystem = new Shooter();
    winchSubsystem = new Winch();
    climbWheelSubsystem = new ClimbWheel();

    // Configuration
    configurePathPlanner();
    configureDriverBindings();
    configureOperatorBindings();
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureDriverBindings() {
    driverController.leftBumper().onTrue(
        new RunCommand(() -> driveSubsystem.lockDrive(), driveSubsystem));

    driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> driveSubsystem.drive(
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstant.DEAD_BAND)),
                SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstant.DEAD_BAND)),
                -SwerveUtils.applyExponentialResponse(MathUtil.applyDeadband(driverController.getRightX(), OperatorConstant.DEAD_BAND)),
                true, true),
            driveSubsystem));
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureOperatorBindings() {
    // Intake note
    operatorController.a().onTrue(new ToggleIntake(intakeSubsystem));

    // AMP note discharge
    operatorController.b().onTrue(
        new SequentialCommandGroup(
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
                .until(() -> elevatorSubsystem.isElevatorAtPosition())));
    // Speaker note shooting
    operatorController.y().onTrue(new SpeakerShot(intakeSubsystem, conveyorSubsystem, shooterSubsystem));

    // Lift arms up and run conveyor until game piece is ready
    operatorController.leftBumper().onTrue(
        new SequentialCommandGroup(
            // Move the note up to the amp position
            new ParallelCommandGroup(
                new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem),
                new RunCommand(() -> intakeSubsystem.runIntake(true), intakeSubsystem),
                new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem))
                .until(() -> conveyorSubsystem.isGamePieceAmpReady()),

            // Stop the conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> shooterSubsystem.stopShooter(), conveyorSubsystem),
                new InstantCommand(() -> conveyorSubsystem.stopConveyor(), intakeSubsystem),
                new InstantCommand(() -> intakeSubsystem.stopIntake(), shooterSubsystem)),

            // Then, move the elevator to climb position
            new RunCommand(() -> elevatorSubsystem.climbPosition(), elevatorSubsystem)
                .until(() -> elevatorSubsystem.isElevatorAtPosition())));

    // Climb and trap score
    operatorController.rightBumper().onTrue(
        new SequentialCommandGroup(
            // Move the elevator to drive position to hook the chain
            new RunCommand(() -> elevatorSubsystem.drivePosition(), elevatorSubsystem)
                .until(() -> elevatorSubsystem.isElevatorAtPosition()),

            // Move the elevator to trap position to score
            new RunCommand(() -> elevatorSubsystem.trapPosition(), elevatorSubsystem)
                .until(() -> elevatorSubsystem.isElevatorAtPosition()),

            // Run the climb wheels to balance, then raise the robot
            new ParallelCommandGroup(
                new RunCommand(() -> climbWheelSubsystem.runClimbWheels(), climbWheelSubsystem),
                new RunCommand(() -> winchSubsystem.raiseRobot(), winchSubsystem))
                .until(() -> winchSubsystem.isRobotAtPosition()),

            // Run the conveyor and shooter again to discharge the game piece
            new ParallelCommandGroup(
                new RunCommand(() -> climbWheelSubsystem.stopClimbWheels(), climbWheelSubsystem),
                new RunCommand(() -> shooterSubsystem.divertGamePiece(), shooterSubsystem),
                new RunCommand(() -> conveyorSubsystem.runConveyorForward(), conveyorSubsystem))
                .until(() -> !conveyorSubsystem.isGamePieceAmpReady()),

            // Finally, stop the conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
                new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem))));
  }

  private void configurePathPlanner() {
    // Register commands
    NamedCommands.registerCommand("ToggleIntake", new ToggleIntake(intakeSubsystem));
    NamedCommands.registerCommand("AmpNoteDischarge", new AmpNoteDischarge(intakeSubsystem, conveyorSubsystem, shooterSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("SpeakerShot", new SpeakerShot(intakeSubsystem, conveyorSubsystem, shooterSubsystem));

    // Build an auto chooser
    autoChooser = AutoBuilder.buildAutoChooser("SpeakerCentre_Quad");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Gets the command to run in autonomous mode.
   *
   * @return The autonomous command to run.`
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
