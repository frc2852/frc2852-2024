package frc.robot;

import frc.robot.Constants.OperatorConstant;
import frc.robot.Constants.SubsystemEnable;
import frc.robot.commands.SpeakerShot;
import frc.robot.commands.ToggleIntake;
import frc.robot.subsystems.PowerHubSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.swerve.SwerveUtils;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private final PowerHubSubsystem powerHubSubsystem;

  private final DriveSubsystem driveSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LEDSubsystem ledSubsystem;

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

    // Initialize controllers with distinct ports
    driverController = new CommandXboxController(OperatorConstant.DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(OperatorConstant.OPERATOR_CONTROLLER_PORT);

    // Initialize helpers
    powerHubSubsystem = new PowerHubSubsystem();

    // Initialize subsystems
    driveSubsystem = initSubsystem(SubsystemEnable.DRIVE, DriveSubsystem::new);
    conveyorSubsystem = initSubsystem(SubsystemEnable.CONVEYOR, ConveyorSubsystem::new);
    elevatorSubsystem = initSubsystem(SubsystemEnable.ELEVATOR, ElevatorSubsystem::new);
    intakeSubsystem = initSubsystem(SubsystemEnable.INTAKE, IntakeSubsystem::new);
    shooterSubsystem = initSubsystem(SubsystemEnable.SHOOTER, ShooterSubsystem::new);
    ledSubsystem = initSubsystem(SubsystemEnable.LED, LEDSubsystem::new);

    // Configuration
    configurePathPlanner();

    if (driveSubsystem != null) {
      configureDriverBindings();
    }

    if (conveyorSubsystem != null && elevatorSubsystem != null && intakeSubsystem != null && shooterSubsystem != null) {
      configureOperatorBindings();
    }
  }

  /**
   * Configures the button bindings for the robot. This method will link input
   * devices to commands.
   */
  private void configureDriverBindings() {
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
    operatorController.a().onTrue(new InstantCommand(intakeSubsystem::toggleIntake, intakeSubsystem));

    // AMP note prepare
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
                .until(() -> elevatorSubsystem.isElevatorAtPosition())));

    // AMP note discharge
    operatorController.x().onTrue(
        new SequentialCommandGroup(
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
    operatorController.y().onTrue(
        new SequentialCommandGroup(
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
            new WaitCommand(0.2),
            // Stop intake, conveyor and shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem),
                new InstantCommand(() -> conveyorSubsystem.stopConveyor(), conveyorSubsystem),
                new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem))));
  }

  private void configurePathPlanner() {
    // Register commands
    NamedCommands.registerCommand("ToggleIntake", new ToggleIntake(intakeSubsystem));
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

  /**
   * Initializes a subsystem if it is enabled.
   *
   * @param <T>         The type of the subsystem that extends {@link SubsystemBase}.
   * @param enabled     A boolean indicating whether the subsystem should be initialized.
   * @param constructor A {@link Supplier} that provides the constructor for the subsystem.
   * @return An instance of the subsystem if enabled, otherwise {@code null}.
   */
  private <T extends SubsystemBase> T initSubsystem(boolean enabled, Supplier<T> constructor) {
    return enabled ? constructor.get() : null;
  }
}
